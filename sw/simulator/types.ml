type size = int
type label = string
type data_layout =
    Bits of (label * size) list
  | Bytes of (label * size) list



type record = data_layout * string * int
let get_record = fun record_layout string offset ->
  (record_layout, string, offset)


(* Little endian *)
let make_int_from_bytes = fun data pos size ->
  if size < 4 then
    let rec mk = fun pos s i ->
      if s = 0 then i else mk (pos+1) (s-1) ((i lsl 8) lor Char.code data.[pos]) in
    mk pos size 0
  else if size = 4 then begin
    let c = fun i -> Int32.shift_left (Int32.of_int (Char.code data.[pos+i])) (8*(3-i)) in
    let lor32 = Int32.logor in
    Int32.to_int (lor32 (c 0) (lor32 (c 1) (lor32 (c 2) (c 3))))
    
  end else invalid_arg "make_int_from_bytes"


(* Little endian *)
let make_int_from_bits = fun data offset pos size ->
  assert(pos < 8);
  assert(size < 31);
  let nb_bits_in_first_byte = min (8-pos) size in
  let i = ((Char.code data.[offset] lsl pos) land 0xff) lsr (8-nb_bits_in_first_byte) in
  let rec mk = fun offset s i ->
    if s = 0
    then i
    else if s < 8
    then (i lsl s) lor (Char.code data.[offset] lsr (8 - s))
    else mk (offset+1) (s-8) ((i lsl 8) lor Char.code data.[offset]) in
  mk (offset+1) (size-nb_bits_in_first_byte) i

let assoc = fun label layout ->
  let rec assoc pos = function
      [] -> failwith ("get_int: unknown field "^label)
    | (l, s)::lss ->
	if l = label
	then (pos, s)
	else assoc (pos + s) lss in
  assoc 0 layout
  

let get_int = fun signed label (record_layout, data, offset) ->
  match record_layout with
    Bits l ->
      let (pos, size) = assoc label l in
      let i = 
      	if pos mod 8 = 0 && size mod 8 = 0 then
	  let pos = pos / 8 and size = size / 8 in
	  make_int_from_bytes data (offset+pos) size
      	else
	  make_int_from_bits data (offset+pos/8) (pos mod 8) size in
      if signed then (i lsl (31-size)) asr (31-size) else i
  | Bytes l ->
      let (pos, size) = assoc label l in
      let i = make_int_from_bytes data (offset+pos) size in
      if size < 4 then
	if signed then (i lsl (31-4*size)) asr (31-4*size) else i
      else begin
	assert(not signed);
	i
      end

let get_int32 = get_int true
let get_u32 = get_int false
let get_uint = get_int false
let get_int = get_int true

let get_raw = fun label (record_layout, data, offset) ->
  match record_layout with
    Bytes layout -> 
      let (pos, size) = assoc label layout in
      String.sub data (offset+pos) size
  | _ -> failwith "get_raw"
  


let sum_sizes = List.fold_left (fun a (_, s) -> a+s) 0
let size_of_message = function (* In bytes *)
    Bytes l -> sum_sizes l
  | Bits l -> sum_sizes l / 8
    
let make_payload = fun layout values ->
  match layout with
    (Bytes layout) ->
      let p = String.create (sum_sizes layout) in
      List.iter
	(fun (label, value) ->
	  let (pos, size) = assoc label layout in
	  let byte = fun x -> Char.chr (x land 0xff) in
	  match size with
	    1 -> p.[pos] <- byte value
	  |	2 ->
	      p.[pos] <- byte (value asr 8);
	      p.[pos+1] <- byte value
	  |	4 ->
	      p.[pos] <- byte (value asr 24);
	      p.[pos+1] <- byte (value lsr 16);
	      p.[pos+2] <- byte (value lsr 8);
	      p.[pos+3] <- byte value
	  |	_ -> failwith "make_payload: unknown int size"
	)
	values;
      p
  | _ -> failwith "make_payload"
  
  
