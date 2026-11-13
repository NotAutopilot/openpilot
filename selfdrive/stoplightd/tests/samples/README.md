# Stoplight samples (Scallywag, comma 3X road camera)

Real frames from Justin's own drives (dongle 1c95345a3286a5db, Pre-AP Model S,
road camera OX03C10 1928x1208). Each file is a 256x256 full-resolution crop
around one signal head, JPEG q92 (4:4:4), base64 so it pushes as text.
`manifest.json` has the route, the route time (s from route start), the crop
origin, and the red / green lamp centers inside the crop.

Every red/green pair is the last red frame (~1.6 s before the switch) and the
first green frame of a stop where the car sat in Drive at 0 m/s and pulled
away on the green. Times below are route start, America/Chicago.

| route | start (CT) | scene |
| --- | --- | --- |
| `000001c2--c42d22cba9` | Oct 23 16:33 | overcast day, US-59 sign beside the head |
| `000001c9--c430fff706` | Oct 24 19:17 | night, rain on the glass, halos |
| `000001ca--33630b538b` | Oct 24 21:16 | night rain, left-turn arrow head |
| `000001d7--578218177b` | Oct 26 17:01 | grey day, dull red LED (chroma ~52) |
| `000001df--a2e9fcf1a2` | Oct 29 10:56 | bright sun, green street sign next to it |
| `000001e1--746e55c63c` | Oct 30 17:40 | dusk, pedestrian head below |
| `000001e3--1ebb4d2a4f` | Oct 30 20:42 | night, dark sky |
| `000001e9--eb89f1fd51` | Oct 31 14:17 | rain, pedestrian head below |
| `000001ed--4568145b1c` | Nov  2 13:23 | fog / grey sky, dull red |
| `000001f7--051a1a1d2d` | Nov  9 17:21 | dusk, right-arrow head, brake lights below |
| `000001f7--051a1a1d2d` | Nov  9 17:21 | **negative**: digital billboard flipping red to green content beside a signal |

`../sequences.json.gz.b64` (gzip + base64 JSON) is the per-frame lamp list (5 Hz) the detector produced on
52 stopped-in-Drive intervals from 60 drives (Oct 23 - Nov 11): 24 stops with a
real red-to-green change, 22 stops with no signal change (parking lots,
garage, car wash, stop signs, the billboard), one right turn on red, and 5
where the signal was out of view.
Positives keep the last 12 s before the car moved (stop 16: all 92 s, it
has two changes); the rest keep up to 60 s. Each lamp is
`[color, x, y, w, h, colored px, core px, chroma, peak luma]`, t is seconds
relative to the car moving off.
