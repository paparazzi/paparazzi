(*
 * Icons library
 *
 * Copyright (C) 2004 CENA/ENAC, Yann Le Fablec
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 *)

let question_icon =
  [|"48 48 69 1";
    "  c     #000000";". c #060708";"X c #06080a";"o c #0a0a0a";"O c #19150d";
    "+ c gray8";"@ c gray10";"# c #221c12";"$ c #393939";"% c #2b3d61";
    "& c #354564";"* c #654f24";"= c #6c5526";"- c #705627";"; c #715929";
    ": c #735f3b";"> c #7c622d";"; c #7f6941";"< c gray39";"1 c #727272";
    "2 c #737579";"3 c gray50";"4 c #81642e";"5 c #876a31";"6 c #8c6d31";
    "7 c #937435";"8 c #9b7b3a";"9 c #a27f3b";"0 c #927c52";"q c #a6823c";
    "w c #a9853d";"e c #a68748";"r c #a38e55";"t c #a48b5a";"y c #b38d40";
    "u c #bc9443";"i c #be9b53";"p c #bea363";"a c #bfa16a";"s c #bea272";
    "d c #c09745";"f c #c39f56";"g c #c9a45b";"h c #d2a64c";"j c #d8ab4e";
    "k c #d8ac5a";"l c #d8b15f";"z c #c3a466";"x c #c6a76a";"c c #c9ac73";
    "v c #d2b06c";"b c #d9b263";"n c #d8b56e";"m c #dbba73";"M c #d8ba7b";
    "N c #f7c35a";"B c #f7c96d";"V c #f7cf7e";"C c #aaaaaa";"Z c #d8be86";
    "A c #dcc494";"S c #f7d48c";"D c #f7d899";"F c #f7dca5";"G c #f7dfaf";
    "H c #f7e2b8";"J c #f7e5c0";"K c white";"L c None";
    "LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLLLLLLo         LLLLLLLLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLLLLL   9uj8q>    LLLLLLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLLLL  jDDHDMMMiq   LLLLLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLLL  NHHDSNNNVMlq=  LLLLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLLo NGFNe77wNNVnlq   LLLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLL pDHN7    7NNBll4  +LLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLo BJV7   %  fNVjlq   LLLLLLLLLLLLLL";
    "LLLLLLLLLLLLL rSGj   &33 tNBljq   KLLLLLLLLLLLLL";
    "LLLLLLLLLLLLL hVNq  &3KK tBBnj4  .CKLLLLLLLLLLLL";
    "LLLLLLLLLLLLL NSNq  2KKL tNnnj*   CKLLLLLLLLLLLL";
    "LLLLLLLLLLLLL vlu4  3KL  zBMjq   3CKLLLLLLLLLLLL";
    "LLLLLLLLLLLLL XX    tKL 6vZlj;   3CKLLLLLLLLLLLL";
    "LLLLLLLLLLLLL  X   X3K  uSZju   +3KLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLL 0<C333. wBZjj    3CKLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLLL 2333K  NSvk8   +3KLLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLLLLLLLL =fAmcaO   CCKLLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLLLLLLLL 5vngx,  .3CKLLLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLLLLLLLL 7Mbks   2CKLLLLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLLLLLLLL uMbk0   3KLLLLLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLLLLLLLL jZlj:  CCKLLLLLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLLLLLLLL iMju   1KLLLLLLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLLLLLLLL eeq4  XCKLLLLLLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLLLLLLLL @$    3CKLLLLLLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLLLLLLLL      X3KLLLLLLLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLLLLLLLLL t2<33CKLLLLLLLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLLLLLLLLL#  X..KLLLLLLLLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLLLLLLLL mVSu   LLLLLLLLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLLLLLLLL VDDj   LLLLLLLLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLLLLLLLL SSlu   CKLLLLLLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLLLLLLLL Snju   CKLLLLLLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLLLLLLLL hlu;  .CKLLLLLLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLLLLLLLLL       CKLLLLLLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLLLLLLLLL      3CKLLLLLLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLLLLLLLLLL XoX33CKLLLLLLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLLLLLLLLLLL 333CKLLLLLLLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL";
    "LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL"|]

let error_icon =
  [|"48 48 133 2";
    "   c #000000";".  c #070909";"X  c #0c0403";"o  c #0a0a0a";"O  c #0d0f10";
    "+  c #130605";"@  c #1d0907";"#  c #1c0908";"$  c #131313";"%  c #1b1b1b";
    "&  c #250c0a";"*  c #2c0e0b";"=  c #310d0a";"-  c #3a0f0b";";  c #35100e";
    ":  c #3d130f";">  c #361210";";  c #242424";"<  c #2c2c2c";"1  c #323232";
    "2  c #3b3b3b";"3  c #41130e";"4  c #401411";"5  c #4d1712";"6  c #4e1814";
    "7  c #571b15";"8  c #591710";"9  c #581b15";"0  c #5c1e1a";"q  c #6e190f";
    "w  c #671c16";"e  c #6b1d15";"r  c #791e15";"t  c #66221d";"y  c #6c231e";
    "u  c #7d231c";"i  c #742620";"p  c #772822";"a  c #7f2821";"s  c #484848";
    "d  c #545454";"f  c #656565";"g  c #7e7e7e";"h  c #951f11";"j  c #981f11";
    "k  c #83241b";"l  c #87281e";"z  c #892116";"x  c #8b261c";"c  c #89281f";
    "v  c #952214";"b  c #932419";"n  c #92291f";"m  c #9c2213";"M  c #9a261a";
    "N  c #9b291d";"B  c #832b24";"V  c #8c2a23";"C  c #8e2e28";"Z  c #8f3832";
    "A  c #942c24";"S  c #9b2d23";"D  c #943029";"F  c #9e3026";"G  c #9d3229";
    "H  c #9e3b32";"J  c #a52415";"K  c #a42618";"L  c #a3291c";"P  c #a82211";
    "I  c #aa2718";"U  c #ab291b";"Y  c #b22413";"T  c #b12718";"R  c #b5291a";
    "E  c #bb2513";"W  c #bc2b1a";"Q  c #a12d22";"!  c #a92f20";"~  c #a03026";
    "^  c #a03229";"/  c #a5392e";"(  c #ab3224";")  c #a9352a";"_  c #a83a2e";
    "`  c #a43c33";"'  c #af3f33";"]  c #c32613";"[  c #c72816";"{  c #c62d1b";
    "}  c #ce2e1c";"|  c #d22813";" . c #d02e1b";".. c #db2a15";"X. c #df301c";
    "o. c #e22b15";"O. c #ec2d15";"+. c #e4311b";"@. c #ea311b";"#. c #ea3a1b";
    "$. c #f4321a";"%. c #f6381c";"&. c #fd3216";"*. c #fb3318";"=. c #fc3b1a";
    "-. c #c1782e";";. c #ff401e";":. c #d48d1d";">. c #cd8622";";. c #c18138";
    "<. c #c79d37";"1. c #c49c39";"2. c #ca9d31";"3. c #cda137";"4. c #cfa43c";
    "5. c #d5a82e";"6. c #dcac2b";"7. c #d4a534";"8. c #d1a53a";"9. c #d8a733";
    "0. c #d9aa32";"q. c #daad3c";"w. c #dfb035";"e. c #ddb039";"r. c #e1ae23";
    "t. c #e1b233";"y. c #cfa540";"u. c #d4aa43";"i. c #d2aa48";"p. c #d8ac40";
    "a. c #dfb445";"s. c #fdfdfd";"d. c None";
    "d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.";
    "d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.";
    "d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.";
    "d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.";
    "d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.";
    "d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.";
    "d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.";
    "d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.";
    "d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.=.&.*.*.$.O.@.@.@.@.@.  d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.";
    "d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.=.=.&.O.| ] E Y Y Y P Y W { } } W   d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.";
    "d.d.d.d.d.d.d.d.d.d.d.d.d.d.&.&...] Y P P P P j v J J J I U R W R     d.d.d.d.d.d.d.d.d.d.d.d.d.";
    "d.d.d.d.d.d.d.d.d.d.d.d.=.&.O.] Y P P P P P m m P I J J K K L U U U z   d.d.d.d.d.d.d.d.d.d.d.d.";
    "d.d.d.d.d.d.d.d.d.d.d.=.&...E P P P P P P v J J J J J I K K L K L L L r     d.d.d.d.d.d.d.d.d.d.";
    "d.d.d.d.d.d.d.d.d.d.=.&.| Y P P P P P P J v J J K I K ) ( K K L K L L N 8     d.d.d.d.d.d.d.d.d.";
    "d.d.d.d.d.d.d.d.d.=.&.| P P P P P P P P J J m m J K K K ( ) L L N L L N k #   d.d.d.d.d.d.d.d.d.";
    "d.d.d.d.d.d.d.d.d.=.o.Y P P P P J P ! ! J I J m m L K K K ) ) ) Q L L Q N w o   d.d.d.d.d.d.d.d.";
    "d.d.d.d.d.d.d.d.=.&.E P P P J P ! ! P ( ( J K K K b m L L K L _ ) Q L L S x ; o   d.d.d.d.d.d.d.";
    "d.d.d.d.d.d.d.d.*.| P P P P P J h S ! K ( ( K K K K b b L L L L ) _ Q Q L S 7       d.d.d.d.d.d.";
    "d.d.d.d.d.d.d.#.*.E P P J h I P m P K ( ( ( ) J K K L b b N Q L L _ _ ( S Q u # $   d.d.d.d.d.d.";
    "d.d.d.d.d.d.d.%...P P P P j J K I I K K ( ( ' ) L L L L M b N Q Q S / _ S S c = o   d.d.d.d.d.d.";
    "d.d.d.d.d.d.  %.[ I :.r.r.r.r.r.r.6.6.6.6.6.e.a.e.0.0.0.0.7.2.2.7.8.8.-.Q S A 3   o   d.d.d.d.d.";
    "d.d.d.d.d.d.  $.E P r.r.r.r.r.r.6.r.6.6.6.6.6.q.e.q.0.9.9.9.9.7.7.8.7.8.Q S A 5 . o   d.d.d.d.d.";
    "d.d.d.d.d.d.  $.E P r.r.r.t.r.r.6.r.6.6.6.0.9.5.3.q.p.9.9.9.7.4.8.8.8.y.~ Q A 7   . , d.d.d.d.d.";
    "d.d.d.d.d.d.  #.Y P r.r.r.r.t.t.6.6.6.6.6.6.6.9.5.2.7.p.p.7.4.7.<.1.1.1.~ F D 9   . % d.d.d.d.d.";
    "d.d.d.d.d.d.  +.T J r.r.6.r.6.0.w.6.6.6.0.0.0.9.9.9.2.7.u.u.u.8.8.8.y.y.^ S C 6     % d.d.d.d.d.";
    "d.d.d.d.d.d.  X.T J r.6.6.6.6.6.e.e.0.0.0.6.2.9.7.7.7.2.3.8.u.u.y.8.4.y.^ G V :     % d.d.d.d.d.";
    "d.d.d.d.d.d.  } R J >.6.6.6.6.6.6.e.e.9.0.9.2.2.9.7.7.3.3.4.8.i.i.4.y.,.^ G a &     % s.d.d.d.d.";
    "d.d.d.d.d.d.  R W K K K I K L N M S ) _ L L Q S S S S S Q S S ^ ^ ` ^ ^ F D w X   . % s.d.d.d.d.";
    "d.d.d.d.d.d.  q { U I K L K L U M b L / Q Q L Q n n Q A Q Q G Q ^ ^ ` G G C 6     o , s.d.d.d.d.";
    "d.d.d.d.d.d.  . U U K K L L L Q L N N N ^ S Q Q S c V A A ^ S G G ^ ^ ` D i # . . o 1 d.d.d.d.d.";
    "d.d.d.d.d.d.  o e R U L L L L L Q L N Q ) / S Q Q S F ~ S ^ ^ G ^ ^ G H Z 6       % d d.d.d.d.d.";
    "d.d.d.d.d.d.d.  . z U L L L L Q L S N N S ` Q S Q ~ Q ^ A A ^ G D / G C t +     o < s.d.d.d.d.d.";
    "d.d.d.d.d.d.d.    + M K L S L Q Q ! Q S S S / Q Q ^ ^ ^ G D ^ ^ ^ G D p *       $ s s.d.d.d.d.d.";
    "d.d.d.d.d.d.d.d.  . - M Q L L Q Q Q Q Q Q Q ` ` V A Q ^ ^ D D ^ G D p >       o , g d.d.d.d.d.d.";
    "d.d.d.d.d.d.d.d.      = x S S Q Q Q Q ~ ~ ~ A ` Z G ^ ^ G G G G C y *         % d s.d.d.d.d.d.d.";
    "d.d.d.d.d.d.d.d.d.1   . @ e N N S S Q Q ~ ~ ~ S H ` G ^ G G D B 0 &         % s s.d.d.d.d.d.d.d.";
    "d.d.d.d.d.d.d.d.d.d.        3 k n S S S F F F G F ` G D D B y 4 .     .   O 1 s.s.d.d.d.d.d.d.d.";
    "d.d.d.d.d.d.d.d.d.d.      .   X ; t k V V A A A C V B p 0 > X           O < g s.d.d.d.d.d.d.d.d.";
    "d.d.d.d.d.d.d.d.d.d.d.        .     # * 4 7 0 9 6 : * #         .     $ 2 g s.d.d.d.d.d.d.d.d.d.";
    "d.d.d.d.d.d.d.d.d.d.d.d.d.      .                         .   .   o , f s.s.d.d.d.d.d.d.d.d.d.d.";
    "d.d.d.d.d.d.d.d.d.d.d.d.d.d.  , %   .     .     .           . $ % d g s.s.d.d.d.d.d.d.d.d.d.d.d.";
    "d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.    $ . . . o       o . O O , s g s.s.d.d.d.d.d.d.d.d.d.d.d.d.d.";
    "d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.  1 , , % % % , , 1 d s.s.s.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.";
    "d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.s.s.s.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.";
    "d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.";
    "d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.";
    "d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.";
    "d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d.d."|]

let warning_icon =
  [|"48 48 97 2";"   c #000000";".  c #0b0501";"X  c #0b0900";"o  c #0a0a0a";
    "O  c #170402";"+  c #120e00";"@  c #1c0705";"#  c #151001";"$  c #1c1501";
    "%  c #131313";"&  c #220704";"*  c #260806";"=  c #2c0805";"-  c #2e0a08";
    ";  c #251d02";":  c #3b0b07";">  c #3d0c08";";  c #2a2103";"<  c #312604";
    "1  c #3a2d05";"2  c gray17";"3  c #410c06";"4  c #440d09";"5  c #4b0d07";
    "6  c #490e09";"7  c #4d100c";"8  c #520e07";"9  c #571109";"0  c #5c110b";
    "q  c #413205";"w  c #4c3b06";"e  c #513f07";"r  c #62120a";"t  c #6d140b";
    "y  c #73150d";"u  c #78150b";"i  c #731711";"p  c #7a1710";"a  c #791812";
    "s  c #524007";"d  c #675009";"f  c #69520a";"g  c #71570a";"h  c #745a0c";
    "j  c #7b610a";"k  c #85170c";"l  c #86180d";"z  c #8a180d";"x  c #851a12";
    "c  c #8a1a11";"v  c #901a0e";"b  c #9a1b0e";"n  c #931d12";"m  c #9c1e14";
    "M  c #a41c0e";"N  c #a91d0f";"B  c #a01d11";"V  c #b31f0f";"C  c #b21f10";
    "Z  c #b9200f";"A  c #b62010";"S  c #bc2111";"D  c #ab4216";"F  c #b14010";
    "G  c #ba5b12";"H  c #bf6f16";"J  c #924f48";"K  c #c62210";"L  c #ca2311";
    "P  c #d32412";"I  c #da2613";"U  c #e32712";"Y  c #e42e1a";"T  c #eb2713";
    "R  c #ec2813";"E  c #f42913";"W  c #c26c13";"Q  c #c67e15";"!  c #c87e14";
    "~  c #b9930e";"^  c #bd9312";"/  c #cb8614";"(  c #cd8912";")  c #c39b0f";
    "_  c #c59913";"`  c #cb9e13";"'  c #d18913";"]  c #d7a816";"[  c #d7a81a";
    "{  c #d8a51c";"}  c #dcaa14";"|  c #d8a919";" . c #dfb112";".. c #e0ad12";
    "X. c #e0b213";"o. c #808080";"O. c None";
    "O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.";
    "O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.";
    "O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.";
    "O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.";
    "O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.";
    "O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.";
    "O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.";
    "O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.R E L c o.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.";
    "O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.T E P N l X   O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.";
    "O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.N E P N M b 0     O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.";
    "O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.N E U V M M B z - %   O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.";
    "O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.R T Z M M A B b r     O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.";
    "O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.N E L M M G W M B v :     O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.";
    "O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.M E I M M M  . .M B B u       O.O.O.O.O.O.O.O.O.O.O.O.O.O.";
    "O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.R T Z M M .... .' B B v 5     O.O.O.O.O.O.O.O.O.O.O.O.O.O.";
    "O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.N E L M M W  . . . .B B B y       O.O.O.O.O.O.O.O.O.O.O.O.O.";
    "O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.N E I N M M ....~ ..} ( M b z 5     O.O.O.O.O.O.O.O.O.O.O.O.O.";
    "O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.U E Z M M ' )   X q } ..M B B y       O.O.O.O.O.O.O.O.O.O.O.O.";
    "O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.M E P M M M  .j . +   ..} ( B m z 6     O.O.O.O.O.O.O.O.O.O.O.O.";
    "O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.T U V M M X. .,     $ } } } B M m y       O.O.O.O.O.O.O.O.O.O.O.";
    "O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.N E L M M ( X. .;     ; } } } ( M m v 6     O.O.O.O.O.O.O.O.O.O.O.";
    "O.O.O.O.O.O.O.O.O.O.O.O.O.O.N R I M M M ..X. .;   ; q } } } } m B b y       O.O.O.O.O.O.O.O.O.O.";
    "O.O.O.O.O.O.O.O.O.O.O.O.O.O.t R S M M ( X. .} w   q q } } } } ( B m c 4     O.O.O.O.O.O.O.O.O.O.";
    "O.O.O.O.O.O.O.O.O.O.O.O.O.N E P N M M  . . ...$   ; q } } } } ] m m m r       O.O.O.O.O.O.O.O.O.";
    "O.O.O.O.O.O.O.O.O.O.O.O.O.U R A M M  . . ...} X   X ; } } } } } ' m m x =     O.O.O.O.O.O.O.O.O.";
    "O.O.O.O.O.O.O.O.O.O.O.O.N R P N M '  . .} } } . + X < } } | } | | m m m 0   %   O.O.O.O.O.O.O.O.";
    "O.O.O.O.O.O.O.O.O.O.O.O.Y R A B M } ........}   +   s } } } | } | ( m m p @     O.O.O.O.O.O.O.O.";
    "O.O.O.O.O.O.O.O.O.O.O.N R P B N '  .} ....} } 1 e + h } } | } | | | m m c 6     O.O.O.O.O.O.O.O.";
    "O.O.O.O.O.O.O.O.O.O.O.Y T Z M B  .} ....} } } ^ f < _ ] | } | } | | / m n i .     O.O.O.O.O.O.O.";
    "O.O.O.O.O.O.O.O.O.O.N R P M B (  .....} } } } } } } | } } | } | | [ | m m c >     O.O.O.O.O.O.O.";
    "O.O.O.O.O.O.O.O.O.O.Y T A B M  .} ..} } } } } ` ; ; _ } | } | | [ | { H m n 0       O.O.O.O.O.O.";
    "O.O.O.O.O.O.O.O.O.M R P M B ( ..} } ..} } } } w     f | } ] | | [ [ { / m m a =     O.O.O.O.O.O.";
    "O.O.O.O.O.O.O.O.O.J R A B M } } } } } } } } } 1   X h | | | | | | | [ [ m m x 7 o o O.O.O.O.O.O.";
    "O.O.O.O.O.O.O.O.M T I M M / ..} } } } } } } } ^ < j ] } | | | | | | [ [ H m m 0 o o   O.O.O.O.O.";
    "O.O.O.O.O.O.O.O.t R K M M m M M B B B G W / / } } } | | | | | | [ [ | { / D m a @   2 O.O.O.O.O.";
    "O.O.O.O.O.O.O.O.3 L B m M M M m B B B B B B m m m m B m m m m m m m m m m m m x >       O.O.O.O.";
    "O.O.O.O.O.O.O.O.O u 5 t t u l z n m m B B B B m m m m m m m m m m m m m m m m m 0       O.O.O.O.";
    "O.O.O.O.O.O.O.O.  o     . & = 3 > 0 0 r t i y i x c x c c c x c x x x x x x a i 7   o   O.O.O.O.";
    "O.O.O.O.O.O.O.O.      o                 . & @ & & 4 > 4 > 4 > > > : : > > > > - @       O.O.O.O.";
    "O.O.O.O.O.O.O.O.              .     o .     o                   o o     o               O.O.O.O.";
    "O.O.O.O.O.O.O.O.O.        o o         .                 .       o       o       o       O.O.O.O.";
    "O.O.O.O.O.O.O.O.O.O.O.O.O.                    o o o                                     O.O.O.O.";
    "O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.                                          O.O.O.O.O.O.";
    "O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.";
    "O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.";
    "O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.";
    "O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.";
    "O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O.O."|]
