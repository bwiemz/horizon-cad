# DWG test drawings

Drawings made by AutoCAD, for the tests of reading DWG (`tests/ui/test_DwgImport.cpp`,
`tests/ui/test_ImportExport.cpp`). They come from GNU LibreDWG's test data, release 0.13.4
(<https://github.com/LibreDWG/libredwg/tree/0.13.4/test/test-data>), and are under its licence,
the GNU General Public License version 3 or later, as Horizon CAD is.

Each holds one entity, drawn by AutoCAD, whose values AutoCAD itself reported. LibreDWG keeps the
report beside each drawing, as `<name>.txt`. The tests check the values against that report, not
against what LibreDWG reads, so a misreading by LibreDWG fails them. The drawings are in inches.

| File | LibreDWG's name | Saved by AutoCAD as | What it holds |
| --- | --- | --- | --- |
| `line_2004.dwg` | `2004/Line.dwg` | 2004 (AC1018) | a line, (21.6078, 2.81192) to (11.0307, 16.6375) |
| `line_2007.dwg` | `2007/Line.dwg` | 2007 (AC1021) | the same line |
| `line_2010.dwg` | `2010/Line.dwg` | 2010 (AC1024) | the same line |
| `line_2013.dwg` | `2013/Line.dwg` | 2013 (AC1027) | the same line |
| `line_2018.dwg` | `2018/Line.dwg` | 2018 (AC1032) | the same line |
| `circle_2018.dwg` | `2018/circle.dwg` | 2018 | a circle, centre (23.4493, -1.09239), radius 13.5304 |
| `arc_2018.dwg` | `2018/Arc.dwg` | 2018 | an arc, centre (-18.1933, 6.68652), radius 8.2931, from 2.00242 to 1.51876 rad |
| `text_2018.dwg` | `2018/Text.dwg` | 2018 | a paragraph of text (MTEXT), 0.2 high |
| `polyline_2018.dwg` | `2018/Polyline.dwg` | 2018 | an open polyline from (9.00535, 12.2913) |
| `ellipse_2018.dwg` | `2018/Ellipse.dwg` | 2018 | an ellipse, centre (11.1433, 9.5374), radii 11.8799 and 3.99294 |
| `spline_2018.dwg` | `2018/Spline.dwg` | 2018 | a spline drawn through three points, from (28.059, 8.80677) |
| `sample_2000.dwg` | `sample_2000.dwg` | 2000 (AC1015) | a small drawing of several entities |
