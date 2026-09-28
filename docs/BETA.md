# Testing a Horizon CAD beta

Thank you for trying a beta. It is built from one commit, and every package
has already been installed on clean Windows, macOS and Linux machines and
started there with `--self-test` (see [Installing](INSTALL.md)). What that
cannot find is what happens when a person uses it, on real hardware: that
is what we ask you to try.

A beta is a **Pre-release** on the
[Releases](https://github.com/bwiemz/horizon-cad/releases) page, tagged
`vX.Y.Z-beta.N`. Install it as [Installing](INSTALL.md) says.

## Before you start

Note down, for your report:

- your system and its version, and your graphics card or chip;
- the package you installed (installer, disk image, AppImage or tarball);
- **Help ▸ About Horizon CAD**: the version, and its source revision (the
  commit it was built from).

Work in a folder of your own, for example `Documents/Horizon beta`. Time
each task roughly: how long it took matters as much as whether it worked.

## Task 1: model and edit a part

1. **File ▸ New Part**.
2. **Model ▸ New Sketch ▸ On the XY Plane**. Draw a rectangle about 60 by 40
   with the **Rectangle** tool (**Draw** tab). Give it a **Distance**
   constraint (**Constrain** tab) on one side, and change its value by
   double-clicking the side.
3. **Model ▸ Finish Sketch**, then **Extrude** it 20.
4. Drill a **Hole** through the top face, and **Fillet** one of the
   vertical edges.
5. Double-click the Extrude in the Feature Tree and make it 30. Check the
   hole and the fillet followed. **Undo**, then **Redo**.
6. **Model ▸ Mass Properties...**: does the volume look right?
7. **File ▸ Save As...** `bracket.hzpart`. Close its tab, then open it again
   with **File ▸ Open**. It should look as you left it.

## Task 2: save and reopen an assembly

1. **File ▸ New Assembly**.
2. **Assembly ▸ Insert Component...** your `bracket.hzpart`, twice.
3. Click a face on each and **Assembly ▸ Add Mate...**: a **Coincident**
   mate, then a second mate of your choice. Drag a component with the
   Select tool: its mates should hold.
4. **File ▸ Save As...** `pair.hzasm`. Close every tab, then open it again.
   The components should be where you left them, with their mates.
5. Open the part (right-click a component, **Open Part**), change it, and
   save it. The assembly should follow.
6. If you have a STEP file from another CAD program, try **File ▸ Import ▸
   STEP as an Assembly...** too, and say how it came in.

## Task 3: create and export a drawing

1. With `bracket.hzpart` open, **Drawing ▸ New Drawing from Part or
   Assembly...**.
2. **Title Block...**: fill in a title and choose a paper size.
3. **Add Dimension**, then click edges on the views: a length, a hole's
   diameter.
4. Save the sheet, close it, and open it again.
5. **File ▸ Export ▸ PDF...**, **SVG...** and **DXF...**. Open each file in
   another program (a PDF viewer, a web browser, another CAD program) and
   check it looks like the sheet.

## Reporting

Please open one issue for each problem, with the
[Beta report](https://github.com/bwiemz/horizon-cad/issues/new?template=beta_report.yml)
form, even for a small one. Attach the file you were working on if you can.
When everything worked, one report saying so, with your system and times, is
just as useful.

If Horizon CAD stops, it offers a crash report the next time it starts;
attach the file it saves. The log is described in
[Installing](INSTALL.md#where-your-files-go).

## Known limits

Please do not report these; they are known:

- A package the release notes list as not signed makes Windows or macOS
  warn before it runs ([Installing](INSTALL.md) says what to do).
- The macOS package is for Apple silicon only.
- On Linux, Horizon CAD draws through X11; on a Wayland desktop that is
  XWayland.
- On a Linux system newer than the package's fontconfig, starting it from a
  terminal prints many `Fontconfig warning` lines. They do not change what
  it does.
- The [Feature Maturity](../README.md#feature-maturity) table in the README
  lists what is experimental, and the STEP entities not yet read.
