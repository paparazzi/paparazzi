#!/usr/bin/env python3
"""Extract the lead melody from an audio file and write a buzzer_note C array.

A passive buzzer can only play one square-wave tone at a time, so the
script:
  1. decodes the audio with ffmpeg (mp3, wav, ogg, ... all work),
  2. follows the highest real voice in every short frame (the tune),
     ignoring overtones and the bass line,
  3. turns stable runs of the same pitch into notes and drops blips,
  4. snaps durations to a rhythmic grid,
  5. moves the melody into the octaves where a buzzer is loud and clear,
  6. separates repeated notes with a short gap so each one is audible.

Requires ffmpeg on the PATH and numpy.
"""
import argparse
import shutil
import subprocess
import wave
from pathlib import Path

import numpy as np

NOTE_NAMES = ["C", "Cs", "D", "Ds", "E", "F", "Fs",
              "G", "Gs", "A", "As", "B"]

SAMPLE_RATE = 22050
FRAME = 2048           # analysis window (~93 ms): short enough to see note gaps
NFFT = 8192            # zero-padded FFT for finer frequency resolution
HOP = 256              # ~11.6 ms between frames
F_LO, F_HI = 80.0, 4200.0   # frequencies considered when looking for notes
PEAK_FLOOR = 0.10      # spectral peaks below 10% of the frame max are ignored
VOICE_RATIO = 0.25     # a voice must be >= 25% as loud as the loudest voice

# Octaves where a typical passive buzzer (resonance ~2-4 kHz) sounds best.
# The melody is shifted by whole octaves so its median lands in this range.
BUZZER_MIDI_LO, BUZZER_MIDI_HI = 72, 107    # C5 .. B7
# Notes are kept down to C4 (quieter on a piezo); buzzer_tunes.c defines C4 .. B7.
BUZZER_TARGET_MEDIAN = 91                   # G6, ~1.6 kHz: tune lands ~1-3 kHz

# The analysis window blurs note edges, so detected gaps between notes come
# out longer than they really are. Give this much back to the note.
GAP_SMEAR_S = 0.045


def midi_to_name(m: int) -> str:
    return f"{NOTE_NAMES[m % 12]}{m // 12 - 1}"


def midi_to_hz(m):
    return 440.0 * 2.0 ** ((np.asarray(m, dtype=float) - 69) / 12)


def decode(path: Path, start: float, duration: float) -> np.ndarray:
    if not shutil.which("ffmpeg"):
        raise SystemExit("ffmpeg not found on PATH (macOS: brew install ffmpeg)")
    cmd = ["ffmpeg", "-v", "error", "-ss", str(start)]
    if duration:
        cmd += ["-t", str(duration)]
    cmd += ["-i", str(path), "-ac", "1", "-ar", str(SAMPLE_RATE),
            "-f", "f32le", "-"]
    raw = subprocess.run(cmd, check=True, capture_output=True).stdout
    audio = np.frombuffer(raw, dtype=np.float32)
    if audio.size < FRAME:
        raise SystemExit("Audio is too short (or the start time is past the end).")
    return audio


def pitch_track(audio: np.ndarray, voicing_db: float, below: int):
    """Follow the highest audible voice (the tune), frame by frame.

    Returns (midi_per_frame, level_per_frame); midi = -1 means rest.
    """
    n_frames = 1 + (audio.size - FRAME) // HOP
    window = np.hanning(FRAME).astype(np.float32)
    idx = np.arange(FRAME)[None, :] + HOP * np.arange(n_frames)[:, None]
    freqs = np.fft.rfftfreq(NFFT, 1 / SAMPLE_RATE)
    band = np.where((freqs >= F_LO) & (freqs <= F_HI))[0]
    b0, b1 = band[0], band[-1]

    midi = np.full(n_frames, -1, dtype=int)
    level = np.zeros(n_frames)
    n_voices = np.zeros(n_frames, dtype=int)

    for s in range(0, n_frames, 512):
        e = min(n_frames, s + 512)
        spec = np.abs(np.fft.rfft(audio[idx[s:e]] * window, n=NFFT, axis=1))
        sub = spec[:, b0 - 1:b1 + 2]
        is_peak = (sub[:, 1:-1] > sub[:, :-2]) & (sub[:, 1:-1] >= sub[:, 2:])
        for r in range(e - s):
            row = sub[r]
            pk = np.nonzero(is_peak[r])[0] + 1
            if pk.size == 0:
                continue
            mags = row[pk]
            keep = mags >= PEAK_FLOOR * mags.max()
            pk, mags = pk[keep], mags[keep]
            # parabolic interpolation for accurate frequency
            l, c, rr = np.log(row[pk - 1] + 1e-12), np.log(mags + 1e-12), np.log(row[pk + 1] + 1e-12)
            off = 0.5 * (l - rr) / (l - 2 * c + rr - 1e-12)
            f = freqs[b0 - 1 + pk] + off * (freqs[1] - freqs[0])

            # drop peaks that are overtones of a lower kept peak
            voices = []          # (freq, mag)
            for fi, mi in sorted(zip(f, mags)):
                is_overtone = False
                for fv, mv in voices:
                    k = round(fi / fv)
                    if k >= 2 and abs(fi / fv - k) < 0.03 * k:
                        is_overtone = True
                        break
                if not is_overtone:
                    voices.append((fi, mi))
            loudest = max(m for _, m in voices)
            voices = [(fv, mv) for fv, mv in voices if mv >= VOICE_RATIO * loudest]
            ftop, mtop = voices[-1]          # highest real voice
            midi[s + r] = int(round(69 + 12 * np.log2(ftop / 440.0)))
            level[s + r] = mtop
            n_voices[s + r] = len(voices)

    # Rests 1: the top voice is much quieter than the loud parts of the song
    ref = np.percentile(level[level > 0], 95) if np.any(level > 0) else 1.0
    midi[level < ref * 10 ** (-voicing_db / 20)] = -1

    # Rests 2: short dips between notes (staccato gaps). A frame whose level
    # drops well below its neighbourhood (+-120 ms) is a gap.
    span = int(0.12 * SAMPLE_RATE / HOP)
    padded = np.pad(level, span, mode="edge")
    local_max = np.lib.stride_tricks.sliding_window_view(padded, 2 * span + 1).max(axis=1)
    midi[level < 0.35 * local_max] = -1

    # Bass: the "top voice" is far below where the tune usually sits, or it
    # is the only thing playing and sits clearly below the tune.
    voiced = midi[midi >= 0]
    if voiced.size:
        tune_center = np.median(voiced)
        low = midi < tune_center - below
        alone_low = (n_voices == 1) & (midi < tune_center - 5)
        midi[(midi >= 0) & (low | alone_low)] = -1
    return midi, level


def median_filter(x: np.ndarray, k: int) -> np.ndarray:
    if k <= 1:
        return x
    pad = k // 2
    xp = np.pad(x, pad, mode="edge")
    windows = np.lib.stride_tricks.sliding_window_view(xp, k)
    return np.median(windows, axis=1).astype(x.dtype)


def segment(midi: np.ndarray, min_note_ms: float):
    """Collapse frame pitches into [midi, start_s, dur_s] notes."""
    hop_s = HOP / SAMPLE_RATE
    notes = []
    start = 0
    for i in range(1, midi.size + 1):
        if i == midi.size or midi[i] != midi[start]:
            notes.append([int(midi[start]), start * hop_s, (i - start) * hop_s])
            start = i

    # Absorb blips shorter than min_note_ms into the louder/longer neighbour
    min_s = min_note_ms / 1000
    changed = True
    while changed and len(notes) > 1:
        changed = False
        k = min(range(len(notes)), key=lambda j: notes[j][2])
        if notes[k][2] >= min_s:
            break
        if k == 0:
            tgt = 1
        elif k == len(notes) - 1:
            tgt = k - 1
        else:
            tgt = k - 1 if notes[k - 1][2] >= notes[k + 1][2] else k + 1
        if tgt < k:
            notes[tgt][2] += notes[k][2]
        else:
            notes[tgt][1] = notes[k][1]
            notes[tgt][2] += notes[k][2]
        del notes[k]
        # Merge neighbours that now have the same pitch
        merged = [notes[0]]
        for n in notes[1:]:
            if n[0] == merged[-1][0]:
                merged[-1][2] += n[2]
            else:
                merged.append(n)
        notes = merged
        changed = True
    return notes


def split_repeated(notes, midi_frames, sal):
    """Split long notes where the energy re-attacks (repeated same pitch)."""
    hop_s = HOP / SAMPLE_RATE
    out = []
    for m, st, du in notes:
        if m < 0:
            out.append([m, st, du])
            continue
        a = int(round(st / hop_s))
        b = int(round((st + du) / hop_s))
        seg = sal[a:b]
        if seg.size < 8:
            out.append([m, st, du])
            continue
        # A new attack = salience jumps up after a dip
        smooth = np.convolve(seg, np.ones(3) / 3, mode="same")
        cuts = []
        min_gap = 6   # frames (~70 ms) between attacks
        for i in range(min_gap, seg.size - min_gap):
            prev_min = smooth[max(0, i - 4):i].min()
            if (smooth[i] > 1.35 * prev_min and smooth[i] >= smooth[i - 1]
                    and smooth[i] >= smooth[min(i + 1, seg.size - 1)]
                    and (not cuts or i - cuts[-1] >= min_gap)):
                # backtrack to the dip
                j = i - 4 + int(np.argmin(smooth[max(0, i - 4):i]))
                if j - (cuts[-1] if cuts else 0) >= min_gap:
                    cuts.append(j)
        edges = [0] + cuts + [seg.size]
        for p, q in zip(edges[:-1], edges[1:]):
            out.append([m, st + p * hop_s, (q - p) * hop_s])
    return out


def estimate_grid_ms(notes) -> float:
    """Guess the basic note spacing (e.g. one 16th note) from onsets.

    For each candidate spacing g, measure how well all note start times sit
    on one common grid of step g (phase coherence). Every subdivision of
    the true spacing also fits, so take the largest g that fits well.
    """
    onsets = np.array([st for m, st, du in notes if m >= 0]) * 1000
    if onsets.size < 4:
        return 200.0
    units = np.arange(60.0, 400.0, 0.5)
    coh = np.abs(np.exp(2j * np.pi * onsets[None, :] / units[:, None]).mean(axis=1))
    good = units[coh >= 0.85 * coh.max()]
    return float(good.max())


def unsmear_gaps(notes):
    """Lengthen notes into a following rest by the window blur."""
    for i in range(len(notes) - 1):
        m, st, du = notes[i]
        nm, nst, ndu = notes[i + 1]
        if m >= 0 and nm < 0:
            give = min(GAP_SMEAR_S, ndu * 0.5)
            notes[i][2] += give
            notes[i + 1][1] += give
            notes[i + 1][2] -= give
    return notes


def quantize(notes, grid_ms: float):
    """Snap note boundaries to a grid; returns [(midi, dur_ms)]."""
    out = []
    t_prev = 0
    for m, st, du in notes:
        end = int(round((st + du) * 1000 / grid_ms))
        if end <= t_prev:
            continue
        out.append([m, end - t_prev])
        t_prev = end
    # merge consecutive rests
    merged = []
    for m, units in out:
        if merged and m < 0 and merged[-1][0] < 0:
            merged[-1][1] += units
        else:
            merged.append([m, units])
    return [(m, units * grid_ms) for m, units in merged]


def fit_to_buzzer(notes, octave_shift):
    """Transpose the whole tune by octaves into the buzzer's loud range.

    The tune is shifted as one block so its shape is preserved; only notes
    that would still fall outside the playable range are folded.
    """
    pitched = [m for m, _ in notes if m >= 0]
    if octave_shift is None:
        med = float(np.median(pitched)) if pitched else BUZZER_TARGET_MEDIAN
        octave_shift = int(round((BUZZER_TARGET_MEDIAN - med) / 12))
    out = []
    for m, d in notes:
        if m >= 0:
            m += 12 * octave_shift
            while m < BUZZER_MIDI_LO - 12:
                m += 12
            while m > BUZZER_MIDI_HI:
                m -= 12
        out.append((m, d))
    return out, octave_shift


def articulate(notes, gap_ms: int):
    """Insert a short rest between identical consecutive notes."""
    out = []
    for i, (m, d) in enumerate(notes):
        nxt = notes[i + 1][0] if i + 1 < len(notes) else None
        if m >= 0 and nxt == m and gap_ms and d > 2 * gap_ms:
            out.append((m, d - gap_ms))
            out.append((-1, gap_ms))
        else:
            out.append((m, d))
    return out


def to_c(notes, name: str) -> str:
    entries = [f"{{{'REST' if m < 0 else midi_to_name(m)}, {int(round(d))}}}"
               for m, d in notes]
    lines = ["  " + ", ".join(entries[i:i + 8]) + ","
             for i in range(0, len(entries), 8)]
    return (f"static const struct buzzer_note {name}[] = {{\n"
            + "\n".join(lines) + "\n  TUNE_END\n};\n")


def write_preview(notes, path: Path):
    """Render a square-wave WAV so you can hear roughly what the buzzer plays."""
    sr = 44100
    chunks = []
    for m, d in notes:
        n = int(sr * d / 1000)
        if m < 0:
            chunks.append(np.zeros(n))
        else:
            t = np.arange(n) / sr
            chunks.append(0.25 * np.sign(np.sin(2 * np.pi * midi_to_hz(m) * t)))
    pcm = (np.concatenate(chunks) * 32767).astype("<i2") if chunks else np.zeros(0, "<i2")
    with wave.open(str(path), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(sr)
        w.writeframes(pcm.tobytes())


def main():
    ap = argparse.ArgumentParser(
        description="Convert an mp3 (or any audio) into a passive-buzzer "
                    "buzzer_note C array.")
    ap.add_argument("input", help="Audio file (mp3, wav, ...)")
    ap.add_argument("-o", "--output",
                    help="Output C file. Default: <input>_buzzer.c")
    ap.add_argument("--start", type=float, default=0.0,
                    help="Start time in seconds. Default: 0")
    ap.add_argument("--length", type=float, default=0.0,
                    help="Seconds of audio to convert (0 = to the end)")
    ap.add_argument("--grid", type=float, default=0.0,
                    help="Note spacing in ms (0 = estimate from the audio)")
    ap.add_argument("--min-note", type=float, default=60.0,
                    help="Drop notes shorter than this many ms. Default: 60")
    ap.add_argument("--octave-shift", type=int, default=None,
                    help="Octaves to transpose (default: auto-fit to buzzer)")
    ap.add_argument("--gap", type=int, default=20,
                    help="Silence in ms between repeated identical notes. Default: 20")
    ap.add_argument("--subdiv", type=int, default=2,
                    help="Timing resolution = note spacing / this. Default: 2")
    ap.add_argument("--voicing", type=float, default=20.0,
                    help="Tune frames this many dB below the loud parts become "
                         "rests. Default: 20")
    ap.add_argument("--below", type=int, default=9,
                    help="Ignore notes more than this many semitones below the "
                         "tune's middle (filters the bass line). Default: 9")
    ap.add_argument("--max-notes", type=int, default=0,
                    help="Truncate to this many entries (0 = no limit)")
    ap.add_argument("--name", default="converted_tune",
                    help="C array name. Default: converted_tune")
    ap.add_argument("--preview", action="store_true",
                    help="Also write <output>.wav with a square-wave preview")
    args = ap.parse_args()

    in_path = Path(args.input).expanduser().resolve()
    if not in_path.is_file():
        raise SystemExit(f"Input file not found: {in_path}")
    out_path = (Path(args.output).expanduser().resolve() if args.output
                else in_path.with_name(in_path.stem + "_buzzer.c"))

    audio = decode(in_path, args.start, args.length)
    midi, sal = pitch_track(audio, args.voicing, args.below)
    midi = median_filter(midi, 5)
    notes = segment(midi, args.min_note)
    notes = split_repeated(notes, midi, sal)
    notes = [n for n in notes if n[2] * 1000 >= args.min_note / 2 or n[0] < 0]

    notes = unsmear_gaps(notes)
    beat = args.grid or estimate_grid_ms(notes)
    # Snap to a fraction of the detected note spacing so short gaps
    # (staccato) survive, but jitter is removed.
    grid = round(beat / args.subdiv)
    qnotes = quantize(notes, grid)
    qnotes, shift = fit_to_buzzer(qnotes, args.octave_shift)
    qnotes = articulate(qnotes, args.gap)
    # leading/trailing silence is useless on a buzzer
    while qnotes and qnotes[0][0] < 0:
        qnotes.pop(0)
    while qnotes and qnotes[-1][0] < 0:
        qnotes.pop()
    if args.max_notes:
        qnotes = qnotes[:args.max_notes]
    if not qnotes:
        raise SystemExit("No melody found. Try a lower --voicing or --min-note.")

    out_path.write_text(to_c(qnotes, args.name), encoding="utf-8")
    total = sum(d for _, d in qnotes) / 1000
    print(f"Converted {in_path}")
    print(f"  note spacing {beat:.0f} ms, grid {grid:.0f} ms, octave shift {shift:+d}, "
          f"{len(qnotes)} entries, {total:.1f} s")
    print(f"Wrote {out_path}")
    if args.preview:
        wav = out_path.with_suffix(".wav")
        write_preview(qnotes, wav)
        print(f"Wrote {wav}")


if __name__ == "__main__":
    main()
