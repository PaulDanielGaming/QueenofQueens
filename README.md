# Mednafen PC-FX — Queen of Queens build

A fork of [pce-devel/mednafenPceDev](https://github.com/pce-devel/mednafenPceDev) carrying five improvements to PC-FX emulation, produced while making the English translation patch for *AJW Queen of Queens*.

Two of them are correctness bugs in Mednafen's PC-FX core that had gone unfound for around twenty years. Every fix here was measured against real hardware captures or a first-principles test, not tuned by eye, and **every claim below comes with a recipe to verify it yourself** from a stock tree.

> **Scope and testing.** This fork was made while producing one translation patch, and **it has only been tested on *AJW Queen of Queens*.** That is a real limitation and worth stating plainly.
>
> The fixes themselves are not game-specific, with one exception. The grey block fix and the colour matrix correction are bugs in Mednafen's RAINBOW and KING code — they affect any PC-FX title using FMV or YUV graphics. The seek latency is general CD behaviour. The NTSC filter is a display option. **Only the confetti repair is shaped around this game**, because it is triggered by encoding defects in these particular video streams.
>
> If you run other PC-FX titles with this build, reports are welcome. Set `pcfx.ntsc off`, `pcfx.chroma_gain 1.0` and `pcfx.gamma 1.0` to get stock behaviour back for comparison.

**Patch:** [romhacking.net](https://www.romhacking.net/translations/7504/) · [archive.org](https://archive.org/details/pauldaniel-PCFX-english-translations)
**Footage:** [youtube.com/@paul_daniel_gaming](https://www.youtube.com/@paul_daniel_gaming)

---

## Contents

1. [Grey block fix](#1-grey-block-fix--rainbowcpp) — discarded quantizer opcodes
2. [Colour matrix](#2-colour-matrix--kingcpp) — wrong YUV→RGB coefficients
3. [NTSC signal simulation](#3-ntsc-signal-simulation--kingcpp--ntsc) — analog bandwidth filter
4. [Confetti bar repair](#4-confetti-bar-repair--rainbowcpp) — corrupt strip detection
5. [CD seek latency](#5-cd-seek-latency--scsicdcpp) — head travel delay
6. [Settings](#settings)
7. [Files changed and building](#files-changed)
8. [What was ruled out](#what-was-ruled-out)

---

## 1. Grey block fix — `rainbow.cpp`

### The symptom

Dark grey rectangles, uniform RGB(43,43,43), at columns 0–1 of strips 9–10 (screen x 0–31, y 144–175) during the Disc B scenario FMV, where real hardware shows black. More broadly: lifted black levels and colour speckle in every dark scene, on both discs.

### The cause

The RAINBOW encoder emits quantizer **rescale** opcodes *after* the last macroblock column of a strip, to set the quantizer up for the *next* strip. Mednafen's strip decoder is:

```c
for(int column = 0; column < 16; column++)
{ ... }
```

It exits the instant `column` reaches 16. Anything still in the bitstream after that is discarded. So the trailing rescale is never read, and the next strip decodes with a stale quantizer.

### The proof

Probing the bits left unread after the loop, frame 99 of the Disc B scenario FMV:

```
strip  6  q0=3 : DC(0) padding only
strip  7  q0=3 : DC(0) padding only
strip  8  q0=2 : RESCALE(q0->3) then padding      <- discarded
strip  9  q0=2 : RESCALE(q0->3) then padding      <- discarded
strip 10  q0=3 : RESCALE(q0->2) RESCALE(q0->3)
strip 11  q0=3 : RESCALE(q0->2) RESCALE(q0->3)
```

Every strip is arranging to hand q0=3 to the next one. Strips already at 3 carry no rescale; strips sitting at 2 carry one that restores 3. Mednafen throws it away, so strips 9 and 10 begin at q0=2 and their leading columns decode one quantizer step too weak. The affected blocks are DC-only, so the quantizer does essentially all the work: dc=−85 at q0=3 clamps to black; the same dc at q0=2 lands at luma 43.

### Independent confirmation from hardware

Output luma is `q × dc / 2 + 128`, so the quantizer a real PC-FX must be using can be solved directly from a hardware capture:

```
q_required = 2 × (hw_luma − 128) / dc
```

| strips | q required by hardware | q Mednafen used | |
|---|---|---|---|
| 4–5 | 1.00 | 1 | agree |
| 6–8 | 3.01 | 3 | agree |
| **9–10** | **3.00** | **2** | **disagree** |
| 11 | 2.98 | 3 | agree |

Hardware independently demands exactly the value the discarded opcodes would have set, and agrees with Mednafen everywhere the opcodes weren't discarded.

### The fix

After the 16-column loop, before chroma interpolation, drain trailing rescales: while bytes remain, peek 9 bits through `dc_y_qlut`; stop at the first code below 0x10 (the zero padding); otherwise consume it and apply the same rescale body `get_dc_coeff` uses.

### Validation

| | before | after |
|---|---|---|
| FMV grey blocks | 95 | **1** |
| BIOS boot logo, cold boot | — | **pixel-identical** |
| Key Tips portrait screen | — | **pixel-identical** |
| hardware error, 45 matched frame pairs | 6.60 | **4.56** |
| artifact region vs hardware | 38.42 | **4.52** |
| rest of frame vs hardware | 5.88 | **4.51** |

The new code executes **zero times** on cold boot, wrestler select, results screen, menus and the Key Tips clip — it is inert on every non-FMV path, which is why those come back pixel-identical.

### Verify it yourself

`docs/RAINBOW_probe_howto.txt` is a self-contained diagnostic you can drop into a **stock** `rainbow.cpp` in about ten minutes. It prints the table above from your own build. No trust required.

---

## 2. Colour matrix — `king.cpp`

### The symptom

Muted red, washed-out saturated blue, muted yellow versus real hardware. Confirmed not a display issue: correct on both CRT and capture.

### The cause

`RebuildUVLUT` uses these coefficients:

```
R = 1.1398 V
G = −0.3946 U − 0.5805 V
B = 2.0320 U
```

Those are exactly the **analog YUV** matrix — the composite-video one, where U and V are pre-scaled by 0.492 and 0.877. But RAINBOW is a motion-JPEG decoder, and JPEG is defined on **full-range BT.601 YCbCr**:

```
R = 1.402 Cr
G = −0.344 Cb − 0.714 Cr
B = 1.772 Cb
```

Red-from-V is 19% low. Blue-from-U is 15% high. This is not a matter of taste; the constants are recognisable on sight to anyone who has implemented either matrix. The conversion also truncated toward zero via an `(int)` cast — the source carried a `FIXME: Use lrint()?` on that exact line.

### The proof — a round trip, no capture needed

Encode a known colour as BT.601 (what a motion-JPEG encoder does), decode it with each matrix, compare to the original:

| colour | target | stock decode | BT.601 decode |
|---|---|---|---|
| saturated red | (255, 0, 0) | (221.5, **19.2**, 0) | (254.9, 0, 0) |
| Aja Kong's singlet red | (190, 40, 45) | (170.4, **51.1**, 39.1) | (190.0, 40.0, 45.0) |
| strong yellow | (235, 215, 40) | (228.6, 222.8, **16.5**) | (235.0, 215.0, 40.1) |
| menu blue | (40, 60, 200) | (45.6, 53.5, **219.0**) | (40.0, 60.0, 199.9) |

**Mean error over ten test colours: stock 9.34, BT.601 0.04.**

BT.601 round-trips exactly. The stock matrix fails in precisely the way the symptoms describe: red loses saturation *and gains green* (muted, pinker); yellow loses red and shifts green; blue overshoots toward white.

### The fix

BT.601 coefficients × an adjustable chroma gain, `lrint` instead of truncation, plus a midtone gamma stage (below 1.0 pulls midtones down while leaving black and white alone — this is what turns bright red into crimson; chroma gain purifies hue but *raises* red, so it cannot do that on its own).

### Validation against a photograph of the real costume

| | R | G | B | G/R | B/R |
|---|---|---|---|---|---|
| WWF photograph of Aja Kong's singlet | 175 | 81 | 64 | 0.460 | 0.368 |
| this build, settled settings | 195 | 86 | 53 | 0.442 | 0.272 |
| stock Mednafen | ~170 | ~105 | ~72 | ~0.62 | ~0.42 |

Green-to-red within two points of the fabric. Skin blue-to-red 0.43 (natural) against 0.34 (orange cast) when gamma was pushed too far — the setting was backed off until skin read correctly, then verified against the photo.

### Verify it yourself

Open `king.cpp`, find `RebuildUVLUT`, read the four coefficients, and compare them to any reference for analog YUV versus BT.601 YCbCr. That's a thirty-second check. The round-trip test is a dozen lines in any language.

---

## 3. NTSC signal simulation — `king.cpp` + `ntsc/`

### The symptom

240p content on a modern display shows hard diagonal edges as a staircase. Worst case: long black hair against a light background. A CRT handles it beautifully; an LCD does not.

### Why scalers cannot fix it

The stepping is in the source pixels — it is the same data a real PC-FX sends to a CRT; the CRT makes it look better on the way out. Interpolation can only average between existing pixels. It cannot reconstruct the intermediate values an analog signal actually carried.

hq2x, SABR and the 2xsai family are worse than useless here. They analyse edges and synthesise new pixels, which suits hand-drawn sprite art. PC-FX FMV is downsampled video and was never meant to be seen as discrete pixels.

### What does fix it

blargg's `snes_ntsc` (LGPL) recreates the analog bandwidth limiting and chroma bleed of a real composite or S-Video path. A hard black-to-white edge comes out as a genuine gradient with real intermediate values — the steps are **filled in**, not blurred.

Measured on a hair edge, steepest single-pixel luma jump:

| | jump | mean slope |
|---|---|---|
| unfiltered | 140.0 | 7.52 |
| S-Video, defaults | 67.0 | 7.82 |
| S-Video, sharpness/resolution −1 | 31.0 | 4.65 |

Mean slope at defaults is essentially unchanged from unfiltered, so overall detail survives while the steepest edges are more than halved.

### How it is wired in

Input is the core's RGB888 line, reduced to RGB16 (rounded, not truncated — a bare mask biases the whole picture darker by about 1%). Output is 32-bit straight back into the surface. Geometry: 3 pixels in, 7 out, so 256 becomes 602; the PC-FX framebuffer is already 1024 wide so nothing reallocates. Only the 256-wide mode is filtered.

Five modes — `off`, `svideo`, `composite`, `rgb`, `monochrome` — and seven tuning values. The tuning values are **offsets from the chosen preset**, so 0.0 means "use the preset as its author intended." (An earlier revision assigned them instead, which clobbered S-Video's own sharpness and switched its fringing back on. Fixed.)

### Verify it yourself

Set `pcfx.ntsc rgb` and look at any long-haired wrestler on the select screen. Then set it `off`. The difference is not subtle.

---

## 4. Confetti bar repair — `rainbow.cpp`

### The symptom

Partial-width coloured noise across one 16-pixel strip for one to three frames during FMV, on reproducible triggers: Manami Toyota's dropkick and bridge-to-pin, Akira Hokuto's body slam, northern lights bomb and tope con hilo.

### The cause

A mid-strip decode desync. **Real hardware shows a milder artifact at the same points** — verified by capture — so the encoding in the game is genuinely defective. The implementations differ in how gracefully they degrade, not in whether they hit bad data.

### The fix

Condemn a strip on either of two independent triggers: the decoded picture measures as noise on luma **and** chroma (chroma matters: the dropkick corruption is coloured and scores near zero on luma alone), or the block runs more than 8 bytes short of its declared size. Repair by repeating that strip from the previous frame. Real PC-FX hardware already pauses video during disc access, so a briefly repeated frame reads as normal.

### Validation

22 repairs across 250,000 strips. Not visible in gameplay; findable only in the log. A magenta test build confirmed every repair reaches the screen and every observed glitch had a matching log entry.

---

## 5. CD seek latency — `scsicd.cpp`

Mednafen jumps between FMV clips instantly. Real hardware moves a laser head, so there is a visible gap. Added a head-travel-proportional delay in `DoREADBase`, reusing the PC Engine CD seek model `get_pce_cd_seek_ms` from `seektime_pce.h`, by Dave Shadoff, scaled by `pcfx.cd_seek_scale`. Game music continues through the gap exactly as on hardware, confirming the delay is scoped to video-clip data reads. Calibrated by side-by-side comparison against a real PC-FX; 0.5–0.7 is the range where the two are indistinguishable by eye.

---

## Settings

All in `mednafen.cfg`, no rebuild needed:

```
pcfx.chroma_gain       0.5–2.0    colour purity, 1.0 = plain BT.601
pcfx.gamma             0.5–2.0    midtone depth, 1.0 = off
pcfx.ntsc              off | svideo | composite | rgb | monochrome
pcfx.ntsc.sharpness   -2.0–2.0    offset from preset; lower softens edges
pcfx.ntsc.resolution  -2.0–2.0    offset; lowering works, raising above 0 does nothing
pcfx.ntsc.brightness  -1.0–1.0    offset; the filter costs ~5%, this compensates
pcfx.ntsc.contrast    -1.0–1.0    offset
pcfx.ntsc.bleed       -2.0–2.0    offset
pcfx.ntsc.fringing    -2.0–2.0    offset
pcfx.cd_seek_scale     0.0–2.0    0 disables the seek delay
```

Any NTSC mode other than `off` widens output from 256 to 602 pixels, so reduce `pcfx.xscale` to suit.

Settled values for an S-Video → RetroTink 2x → CRT reference (see `settled_config/`):

```
pcfx.ntsc rgb
pcfx.ntsc.sharpness -0.5
pcfx.ntsc.resolution -0.5
pcfx.ntsc.brightness 0.06
pcfx.gamma 0.84
pcfx.chroma_gain 1.04
pcfx.scanlines 0
```

`rgb` rather than `svideo` because the RetroTink separates luma and chroma digitally, so the composite artifacts `svideo` simulates never occur in that chain. Scanlines off because a line-doubler fills every line.

**Two warnings.** Mednafen writes every setting back to the cfg on exit, including command-line flags — there is no temporary override. And an unset setting is stored as `name ` with a trailing space that *is* the value separator; stripping trailing whitespace globally makes thousands of lines unparseable.

---

## Files changed

```
mednafen/src/pcfx/king.cpp        colour matrix, gamma, NTSC hook
mednafen/src/pcfx/pcfx.cpp        setting registrations
mednafen/src/pcfx/rainbow.cpp     confetti repair, grey block fix
mednafen/src/pcfx/ntsc/           snes_ntsc (LGPL, Shay Green)
mednafen/src/cdrom/scsicd.cpp     seek latency
```

Everything else is stock. `snes_ntsc` is `#include`d into `king.cpp` rather than compiled separately, so **no build-system change is needed** — the CI runs `configure` and `make` without `autoreconf`, and a `Makefile.am` edit would be silently ignored.

Build exactly as upstream: see `README_linux_compile.md`, `README_Windows_compile.md`, `README_macOS_compile.md`. `build_windows.sh` cross-compiles the Windows binary on Linux using the same Fedora container recipe the CI uses.

---

## What was ruled out

Recorded so nobody re-treads it.

**Grey blocks — eleven failed attempts before the cause was found.** Per-strip quantizer reset wrecked the BIOS logo (the base table is zero until the first 0xFF block). Resetting or restoring the quantizer on null runs destroyed every RAINBOW-decoded still, because stills are full of null runs. Y-only strip-local rescales posterised the BIOS logo, proving quantizer carry across strips is mandatory. Rounding changes, scaling changes, int16-overflow theories — all tested, all disproven. The lesson: **null runs must not touch the quantizer, and carry across strips is mandatory.** The answer came from reading the unread bitstream directly, not from more theorising.

**Global speed drift (~1s over a two-minute FMV).** Measured the core directly: it reports 358,994.1 master cycles per frame against an ideal of 358,995 (263 lines × 1365) — three parts per million, 0.0004s over two minutes. Master clock correct at 21477272.72. Audio resampler tolerance 9×10⁻⁷. FMV advance divisor matches hardware. **The PC-FX core is not the cause.** If the drift is real it lives in Mednafen's generic frontend pacing loop or in the original stopwatch measurement. Core timing work will not fix it.

**Washed-out blue on VDC menu text.** Deliberately left alone. The blue channel is already pinned at 254; the text reads pale because red and green are also high. Chroma gain cannot reach it. Candidates — the 4-bit palette chroma expansion reaching only 88% of full range, Y read too wide, or the palette format not being Y:8/U:4/V:4 — are indistinguishable without a hardware reference. The palette path feeds every VDC layer in every PC-FX game, so changing it on a guess is high blast radius with no oracle.

**Block-boundary over-read.** `bits_bytes_left` counts physical bytes, but the fetch decrements by one while consuming two on a stuffed 0xFF pair. 18.6% of strips over-read by up to 15 bytes. Charging the stuffed byte takes it to zero — but output is byte-identical everywhere testable, so it was not shipped. Noted for anyone working on the starvation path.

---

## Documentation

- `docs/MEDNAFEN_QoQ_CHANGE_LOG.txt` — the full record: every fix, every measurement, every rejected approach
- `docs/RAINBOW_trailing_rescale_bug.txt` — the grey block bug written up for other developers
- `docs/RAINBOW_probe_howto.txt` — reproduce the grey block finding from any build in ten minutes
- `docs/QoQ_PATCH_TECHNICAL_RECORD.txt` — the game side: font offsets, text structure, the results-screen kanji
- `settled_config/` — the working cfg and a summary of why each value is what it is

---

## Licence

Mednafen is GPL-2, © Mednafen Team. `snes_ntsc` is LGPL, © 2006-2007 Shay Green. The PC Engine CD seek model in `seektime_pce.h`, which the seek latency builds on, is © 2022 Dave Shadoff. PC-FX modifications are offered under the same terms as the code they modify.

— Paul Daniel
