#pragma once

// STEP text for the reader's tests, written by hand as other systems write their solids.
// Shared by test_StepFormat, test_StepCurved, test_StepExact, test_ExactMass, test_ExactTransform
// and test_ExactValidator.

#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "horizon/math/Vec3.h"

namespace hz::step_text {

/// @p data as the DATA section of a Part-21 file, @p units before it.
inline std::string stepFile(const std::string& data, const std::string& units = {}) {
    return "ISO-10303-21;\nHEADER;\nFILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\n" + units +
           data + "ENDSEC;\nEND-ISO-10303-21;\n";
}

/// A foreign-style cylinder (radius 1, height 2, axis +Z) the way OCC-based
/// systems export it: CIRCLE rim edges (closed, seam vertex NOT on the
/// placement X-axis), a seam LINE wrapped in SURFACE_CURVE, PLANE caps —
/// the bottom cap with an inward normal declared via same_sense = .F. —
/// and a CYLINDRICAL_SURFACE lateral face.
inline std::string externalStyleCylinder() {
    return R"(ISO-10303-21;
HEADER;
FILE_SCHEMA(('AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF { 1 0 10303 442 3 1 4 }'));
ENDSEC;
DATA;
#1 = CARTESIAN_POINT('',(0.,0.,0.));
#2 = DIRECTION('',(0.,0.,1.));
#3 = DIRECTION('',(0.,1.,0.));
#4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);
#5 = CIRCLE('',#4,1.);
#6 = CARTESIAN_POINT('',(1.,0.,0.));
#7 = VERTEX_POINT('',#6);
#8 = CARTESIAN_POINT('',(0.,0.,2.));
#9 = DIRECTION('',(1.,0.,0.));
#10 = AXIS2_PLACEMENT_3D('',#8,#2,#9);
#11 = CIRCLE('',#10,1.);
#12 = CARTESIAN_POINT('',(1.,0.,2.));
#13 = VERTEX_POINT('',#12);
#14 = EDGE_CURVE('',#7,#7,#5,.T.);
#15 = EDGE_CURVE('',#13,#13,#11,.T.);
#16 = LINE('',#6,VECTOR('',#2,1.));
#17 = SURFACE_CURVE('',#16,(),.CURVE_3D.);
#18 = EDGE_CURVE('',#7,#13,#17,.T.);
#19 = AXIS2_PLACEMENT_3D('',#1,#2,#9);
#20 = CYLINDRICAL_SURFACE('',#19,1.);
#21 = ORIENTED_EDGE('',*,*,#14,.T.);
#22 = ORIENTED_EDGE('',*,*,#18,.T.);
#23 = ORIENTED_EDGE('',*,*,#15,.F.);
#24 = ORIENTED_EDGE('',*,*,#18,.F.);
#25 = EDGE_LOOP('',(#21,#22,#23,#24));
#26 = FACE_OUTER_BOUND('',#25,.T.);
#27 = ADVANCED_FACE('',(#26),#20,.T.);
#28 = AXIS2_PLACEMENT_3D('',#8,#2,$);
#29 = PLANE('',#28);
#30 = ORIENTED_EDGE('',*,*,#15,.T.);
#31 = EDGE_LOOP('',(#30));
#32 = FACE_OUTER_BOUND('',#31,.T.);
#33 = ADVANCED_FACE('',(#32),#29,.T.);
#34 = AXIS2_PLACEMENT_3D('',#1,#2,$);
#35 = PLANE('',#34);
#36 = ORIENTED_EDGE('',*,*,#14,.F.);
#37 = EDGE_LOOP('',(#36));
#38 = FACE_OUTER_BOUND('',#37,.T.);
#39 = ADVANCED_FACE('',(#38),#35,.F.);
#40 = CLOSED_SHELL('',(#27,#33,#39));
#41 = MANIFOLD_SOLID_BREP('cyl',#40);
ENDSEC;
END-ISO-10303-21;
)";
}

/// A cone: base radius 2 at z = 0, apex at z = 3; the side a face from the
/// base circle up a seam to the apex and back. @p semiAngle as written.
inline std::string cone(const std::string& semiAngle = "0.5880026035475675",
                        const std::string& units = {}) {
    return stepFile(R"(#1 = CARTESIAN_POINT('',(0.,0.,0.));
#2 = DIRECTION('',(0.,0.,1.));
#3 = DIRECTION('',(1.,0.,0.));
#4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);
#5 = CIRCLE('',#4,2.);
#6 = CARTESIAN_POINT('',(2.,0.,0.));
#7 = VERTEX_POINT('',#6);
#8 = CARTESIAN_POINT('',(0.,0.,3.));
#9 = VERTEX_POINT('',#8);
#10 = EDGE_CURVE('',#7,#7,#5,.T.);
#11 = DIRECTION('',(-0.55470019622522915,0.,0.83205029433784372));
#12 = VECTOR('',#11,1.);
#13 = LINE('',#6,#12);
#14 = EDGE_CURVE('',#7,#9,#13,.T.);
#15 = DIRECTION('',(0.,0.,-1.));
#16 = AXIS2_PLACEMENT_3D('',#1,#15,#3);
#17 = CONICAL_SURFACE('',#16,2.,)" +
                        semiAngle + R"();
#18 = ORIENTED_EDGE('',*,*,#10,.T.);
#19 = ORIENTED_EDGE('',*,*,#14,.T.);
#20 = ORIENTED_EDGE('',*,*,#14,.F.);
#21 = EDGE_LOOP('',(#18,#19,#20));
#22 = FACE_OUTER_BOUND('',#21,.T.);
#23 = ADVANCED_FACE('',(#22),#17,.T.);
#24 = PLANE('',#4);
#25 = ORIENTED_EDGE('',*,*,#10,.F.);
#26 = EDGE_LOOP('',(#25));
#27 = FACE_OUTER_BOUND('',#26,.T.);
#28 = ADVANCED_FACE('',(#27),#24,.F.);
#29 = CLOSED_SHELL('',(#23,#28));
#30 = MANIFOLD_SOLID_BREP('cone',#29);
)",
                    units);
}

/// A frustum from radius 1 at z = 0 to radius 2 at z = 2, written as externalStyleCylinder()
/// writes its cylinder: a CONICAL_SURFACE side between two rims and a seam, two PLANE caps.
inline std::string frustum() {
    return stepFile(R"(#1 = CARTESIAN_POINT('',(0.,0.,0.));
#2 = DIRECTION('',(0.,0.,1.));
#3 = DIRECTION('',(0.,1.,0.));
#4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);
#5 = CIRCLE('',#4,1.);
#6 = CARTESIAN_POINT('',(1.,0.,0.));
#7 = VERTEX_POINT('',#6);
#8 = CARTESIAN_POINT('',(0.,0.,2.));
#9 = DIRECTION('',(1.,0.,0.));
#10 = AXIS2_PLACEMENT_3D('',#8,#2,#9);
#11 = CIRCLE('',#10,2.);
#12 = CARTESIAN_POINT('',(2.,0.,2.));
#13 = VERTEX_POINT('',#12);
#14 = EDGE_CURVE('',#7,#7,#5,.T.);
#15 = EDGE_CURVE('',#13,#13,#11,.T.);
#50 = DIRECTION('',(0.44721359549995793,0.,0.89442719099991586));
#16 = LINE('',#6,VECTOR('',#50,1.));
#17 = SURFACE_CURVE('',#16,(),.CURVE_3D.);
#18 = EDGE_CURVE('',#7,#13,#17,.T.);
#19 = AXIS2_PLACEMENT_3D('',#1,#2,#9);
#20 = CONICAL_SURFACE('',#19,1.,0.46364760900080612);
#21 = ORIENTED_EDGE('',*,*,#14,.T.);
#22 = ORIENTED_EDGE('',*,*,#18,.T.);
#23 = ORIENTED_EDGE('',*,*,#15,.F.);
#24 = ORIENTED_EDGE('',*,*,#18,.F.);
#25 = EDGE_LOOP('',(#21,#22,#23,#24));
#26 = FACE_OUTER_BOUND('',#25,.T.);
#27 = ADVANCED_FACE('',(#26),#20,.T.);
#28 = AXIS2_PLACEMENT_3D('',#8,#2,$);
#29 = PLANE('',#28);
#30 = ORIENTED_EDGE('',*,*,#15,.T.);
#31 = EDGE_LOOP('',(#30));
#32 = FACE_OUTER_BOUND('',#31,.T.);
#33 = ADVANCED_FACE('',(#32),#29,.T.);
#34 = AXIS2_PLACEMENT_3D('',#1,#2,$);
#35 = PLANE('',#34);
#36 = ORIENTED_EDGE('',*,*,#14,.F.);
#37 = EDGE_LOOP('',(#36));
#38 = FACE_OUTER_BOUND('',#37,.T.);
#39 = ADVANCED_FACE('',(#38),#35,.F.);
#40 = CLOSED_SHELL('',(#27,#33,#39));
#41 = MANIFOLD_SOLID_BREP('frustum',#40);
)");
}

/// A sphere of radius 1.5: one face, its boundary a seam from pole to pole
/// and back.
inline std::string sphere() {
    return stepFile(R"(#1 = CARTESIAN_POINT('',(0.,0.,0.));
#2 = DIRECTION('',(0.,0.,1.));
#3 = DIRECTION('',(1.,0.,0.));
#4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);
#5 = SPHERICAL_SURFACE('',#4,1.5);
#6 = CARTESIAN_POINT('',(0.,0.,-1.5));
#7 = VERTEX_POINT('',#6);
#8 = CARTESIAN_POINT('',(0.,0.,1.5));
#9 = VERTEX_POINT('',#8);
#10 = DIRECTION('',(0.,-1.,0.));
#11 = AXIS2_PLACEMENT_3D('',#1,#10,#3);
#12 = CIRCLE('',#11,1.5);
#13 = EDGE_CURVE('',#7,#9,#12,.T.);
#14 = ORIENTED_EDGE('',*,*,#13,.T.);
#15 = ORIENTED_EDGE('',*,*,#13,.F.);
#16 = EDGE_LOOP('',(#14,#15));
#17 = FACE_OUTER_BOUND('',#16,.T.);
#18 = ADVANCED_FACE('',(#17),#5,.T.);
#19 = CLOSED_SHELL('',(#18));
#20 = MANIFOLD_SOLID_BREP('sphere',#19);
)");
}

/// A lune of a ball of radius 3 about the origin, a quarter turn from the half-plane y = 0
/// (x > 0) round to x = 0 (y > 0), with its two half-disc caps, as a partial revolve of a
/// semicircle makes it: its sphere bounded by the two meridians, each from pole to pole, with no
/// seam. Its sphere is placed with its seam at 45 degrees, inside the lune, as another system may
/// place it: read into [0, 2 pi), its meridians' pcurves lie either side of the seam, and only
/// the face's sense says which way round the poles its loop runs.
inline std::string lune() {
    return stepFile(R"(#1 = CARTESIAN_POINT('',(0.,0.,0.));
#2 = CARTESIAN_POINT('',(0.,0.,-3.));
#3 = VERTEX_POINT('',#2);
#4 = CARTESIAN_POINT('',(0.,0.,3.));
#5 = VERTEX_POINT('',#4);
#6 = DIRECTION('',(0.,-1.,0.));
#7 = DIRECTION('',(0.,0.,-1.));
#8 = AXIS2_PLACEMENT_3D('',#1,#6,#7);
#9 = CIRCLE('',#8,3.);
#10 = EDGE_CURVE('',#3,#5,#9,.T.);
#11 = DIRECTION('',(1.,0.,0.));
#12 = AXIS2_PLACEMENT_3D('',#1,#11,#7);
#13 = CIRCLE('',#12,3.);
#14 = EDGE_CURVE('',#3,#5,#13,.T.);
#15 = DIRECTION('',(0.,0.,1.));
#16 = VECTOR('',#15,1.);
#17 = LINE('',#2,#16);
#18 = EDGE_CURVE('',#3,#5,#17,.T.);
#19 = ORIENTED_EDGE('',*,*,#14,.T.);
#20 = ORIENTED_EDGE('',*,*,#10,.F.);
#21 = EDGE_LOOP('',(#19,#20));
#22 = FACE_OUTER_BOUND('',#21,.T.);
#23 = DIRECTION('',(0.7071067811865476,0.7071067811865475,0.));
#24 = AXIS2_PLACEMENT_3D('',#1,#15,#23);
#25 = SPHERICAL_SURFACE('',#24,3.);
#26 = ADVANCED_FACE('',(#22),#25,.T.);
#27 = ORIENTED_EDGE('',*,*,#10,.T.);
#28 = ORIENTED_EDGE('',*,*,#18,.F.);
#29 = EDGE_LOOP('',(#27,#28));
#30 = FACE_OUTER_BOUND('',#29,.T.);
#31 = AXIS2_PLACEMENT_3D('',#1,#6,#11);
#32 = PLANE('',#31);
#33 = ADVANCED_FACE('',(#30),#32,.T.);
#34 = ORIENTED_EDGE('',*,*,#14,.F.);
#35 = ORIENTED_EDGE('',*,*,#18,.T.);
#36 = EDGE_LOOP('',(#34,#35));
#37 = FACE_OUTER_BOUND('',#36,.T.);
#38 = DIRECTION('',(-1.,0.,0.));
#39 = DIRECTION('',(0.,1.,0.));
#40 = AXIS2_PLACEMENT_3D('',#1,#38,#39);
#41 = PLANE('',#40);
#42 = ADVANCED_FACE('',(#37),#41,.T.);
#43 = CLOSED_SHELL('',(#26,#33,#42));
#44 = MANIFOLD_SOLID_BREP('lune',#43);
)");
}

/// A cone of radius 2.5 and height 10, its apex at the origin and its base below, as Unigraphics
/// writes one (the Moon buggy's face #32800): its CONICAL_SURFACE of radius 0 placed at the apex,
/// along -z; its side bounded by the base's circle and a VERTEX_LOOP at the apex; a flat base.
inline std::string apexCone() {
    return stepFile(R"(#1 = CARTESIAN_POINT('',(0.,0.,-10.));
#2 = DIRECTION('',(0.,0.,1.));
#3 = DIRECTION('',(1.,0.,0.));
#4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);
#5 = CIRCLE('',#4,2.5);
#6 = CARTESIAN_POINT('',(2.5,0.,-10.));
#7 = VERTEX_POINT('',#6);
#8 = CARTESIAN_POINT('',(0.,0.,0.));
#9 = VERTEX_POINT('',#8);
#10 = EDGE_CURVE('',#7,#7,#5,.T.);
#11 = ORIENTED_EDGE('',*,*,#10,.F.);
#12 = EDGE_LOOP('',(#11));
#13 = FACE_BOUND('',#12,.T.);
#14 = DIRECTION('',(0.,0.,-1.));
#15 = DIRECTION('',(-1.,0.,0.));
#16 = AXIS2_PLACEMENT_3D('',#1,#14,#15);
#17 = PLANE('',#16);
#18 = ADVANCED_FACE('',(#13),#17,.T.);
#19 = ORIENTED_EDGE('',*,*,#10,.T.);
#20 = EDGE_LOOP('',(#19));
#21 = FACE_BOUND('',#20,.T.);
#22 = VERTEX_LOOP('',#9);
#23 = FACE_BOUND('',#22,.T.);
#24 = DIRECTION('',(-1.,0.,-4.44089209850063E-16));
#25 = AXIS2_PLACEMENT_3D('',#8,#14,#24);
#26 = CONICAL_SURFACE('',#25,0.,0.24497866312686414);
#27 = ADVANCED_FACE('',(#21,#23),#26,.T.);
#28 = CLOSED_SHELL('',(#18,#27));
#29 = MANIFOLD_SOLID_BREP('apex cone',#28);
)");
}

/// A ball of radius 10 about the origin as Unigraphics writes one (the Moon buggy's face #32040):
/// one face, bounded by a VERTEX_LOOP at its north pole alone, its placement's directions as that
/// file has them (x not of unit length, z a hair off the axis).
inline std::string vertexLoopBall() {
    return stepFile(R"(#1 = CARTESIAN_POINT('',(0.,0.,10.));
#2 = VERTEX_POINT('',#1);
#3 = VERTEX_LOOP('',#2);
#4 = FACE_BOUND('',#3,.T.);
#5 = CARTESIAN_POINT('',(0.,0.,0.));
#6 = DIRECTION('',(1.09273919746571E-16,0.,1.));
#7 = DIRECTION('',(39.3700787401575,0.,0.));
#8 = AXIS2_PLACEMENT_3D('',#5,#6,#7);
#9 = SPHERICAL_SURFACE('',#8,10.);
#10 = ADVANCED_FACE('',(#4),#9,.T.);
#11 = CLOSED_SHELL('',(#10));
#12 = MANIFOLD_SOLID_BREP('ball',#11);
)");
}

/// A drill's point on a rod: a cylinder of radius 3 from z = 0 up to z = 10, its top flat, and
/// below z = 0 a cone of half-angle 59 degrees (a 118 degree point) to its apex on the axis, as
/// OpenCASCADE writes one: each curved face bounded by its rims and a seam, the cone's up to the
/// apex and back.
inline std::string drillPoint() {
    return stepFile(R"(#1 = CARTESIAN_POINT('',(0.,0.,0.));
#2 = DIRECTION('',(0.,0.,1.));
#3 = DIRECTION('',(1.,0.,0.));
#4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);
#5 = CARTESIAN_POINT('',(0.,0.,10.));
#6 = AXIS2_PLACEMENT_3D('',#5,#2,#3);
#7 = CIRCLE('',#4,3.);
#8 = CIRCLE('',#6,3.);
#9 = CARTESIAN_POINT('',(3.,0.,0.));
#10 = VERTEX_POINT('',#9);
#11 = CARTESIAN_POINT('',(3.,0.,10.));
#12 = VERTEX_POINT('',#11);
#13 = CARTESIAN_POINT('',(0.,0.,-1.8025818570826808));
#14 = VERTEX_POINT('',#13);
#15 = EDGE_CURVE('',#10,#10,#7,.T.);
#16 = EDGE_CURVE('',#12,#12,#8,.T.);
#17 = LINE('',#9,VECTOR('',#2,1.));
#18 = EDGE_CURVE('',#10,#12,#17,.T.);
#19 = DIRECTION('',(-0.85716730070211233,0.,-0.51503807491005416));
#20 = LINE('',#9,VECTOR('',#19,1.));
#21 = EDGE_CURVE('',#10,#14,#20,.T.);
#22 = CYLINDRICAL_SURFACE('',#4,3.);
#23 = ORIENTED_EDGE('',*,*,#15,.T.);
#24 = ORIENTED_EDGE('',*,*,#18,.T.);
#25 = ORIENTED_EDGE('',*,*,#16,.F.);
#26 = ORIENTED_EDGE('',*,*,#18,.F.);
#27 = EDGE_LOOP('',(#23,#24,#25,#26));
#28 = FACE_OUTER_BOUND('',#27,.T.);
#29 = ADVANCED_FACE('',(#28),#22,.T.);
#30 = PLANE('',#6);
#31 = ORIENTED_EDGE('',*,*,#16,.T.);
#32 = EDGE_LOOP('',(#31));
#33 = FACE_OUTER_BOUND('',#32,.T.);
#34 = ADVANCED_FACE('',(#33),#30,.T.);
#35 = CONICAL_SURFACE('',#4,3.,1.0297442586766543);
#36 = ORIENTED_EDGE('',*,*,#15,.F.);
#37 = ORIENTED_EDGE('',*,*,#21,.T.);
#38 = ORIENTED_EDGE('',*,*,#21,.F.);
#39 = EDGE_LOOP('',(#36,#37,#38));
#40 = FACE_OUTER_BOUND('',#39,.T.);
#41 = ADVANCED_FACE('',(#40),#35,.T.);
#42 = CLOSED_SHELL('',(#29,#34,#41));
#43 = MANIFOLD_SOLID_BREP('drill point',#42);
)");
}

/// A torus of radii 4 and 1: one face, its boundary two seams, each there
/// and back.
inline std::string torus() {
    return stepFile(R"(#1 = CARTESIAN_POINT('',(0.,0.,0.));
#2 = DIRECTION('',(0.,0.,1.));
#3 = DIRECTION('',(1.,0.,0.));
#4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);
#5 = TOROIDAL_SURFACE('',#4,4.,1.);
#6 = CARTESIAN_POINT('',(5.,0.,0.));
#7 = VERTEX_POINT('',#6);
#8 = CIRCLE('',#4,5.);
#9 = EDGE_CURVE('',#7,#7,#8,.T.);
#10 = CARTESIAN_POINT('',(4.,0.,0.));
#11 = DIRECTION('',(0.,-1.,0.));
#12 = AXIS2_PLACEMENT_3D('',#10,#11,#3);
#13 = CIRCLE('',#12,1.);
#14 = EDGE_CURVE('',#7,#7,#13,.T.);
#15 = ORIENTED_EDGE('',*,*,#9,.T.);
#16 = ORIENTED_EDGE('',*,*,#14,.T.);
#17 = ORIENTED_EDGE('',*,*,#9,.F.);
#18 = ORIENTED_EDGE('',*,*,#14,.F.);
#19 = EDGE_LOOP('',(#15,#16,#17,#18));
#20 = FACE_OUTER_BOUND('',#19,.T.);
#21 = ADVANCED_FACE('',(#20),#5,.T.);
#22 = CLOSED_SHELL('',(#21));
#23 = MANIFOLD_SOLID_BREP('torus',#22);
)");
}

/// A cylinder of radius 1 and height 2 whose side is bounded by its two
/// rims and no seam, as Solid Edge, Alibre, NX and Onshape write one: two
/// loops that each go once round it. @p surfaceRadius is its surface's,
/// which may be a little off the rims' circles, as another system's is.
inline std::string bandCylinder(const std::string& surfaceRadius = "1.") {
    return stepFile(R"(#1 = CARTESIAN_POINT('',(0.,0.,0.));
#2 = DIRECTION('',(0.,0.,1.));
#3 = DIRECTION('',(0.,1.,0.));
#4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);
#5 = CIRCLE('',#4,1.);
#6 = CARTESIAN_POINT('',(1.,0.,0.));
#7 = VERTEX_POINT('',#6);
#8 = CARTESIAN_POINT('',(0.,0.,2.));
#9 = DIRECTION('',(1.,0.,0.));
#10 = AXIS2_PLACEMENT_3D('',#8,#2,#9);
#11 = CIRCLE('',#10,1.);
#12 = CARTESIAN_POINT('',(1.,0.,2.));
#13 = VERTEX_POINT('',#12);
#14 = EDGE_CURVE('',#7,#7,#5,.T.);
#15 = EDGE_CURVE('',#13,#13,#11,.T.);
#19 = AXIS2_PLACEMENT_3D('',#1,#2,#9);
#20 = CYLINDRICAL_SURFACE('',#19,)" +
                    surfaceRadius + R"();
#21 = ORIENTED_EDGE('',*,*,#14,.T.);
#22 = EDGE_LOOP('',(#21));
#23 = FACE_OUTER_BOUND('',#22,.T.);
#24 = ORIENTED_EDGE('',*,*,#15,.F.);
#25 = EDGE_LOOP('',(#24));
#26 = FACE_BOUND('',#25,.T.);
#27 = ADVANCED_FACE('',(#23,#26),#20,.T.);
#28 = AXIS2_PLACEMENT_3D('',#8,#2,$);
#29 = PLANE('',#28);
#30 = ORIENTED_EDGE('',*,*,#15,.T.);
#31 = EDGE_LOOP('',(#30));
#32 = FACE_OUTER_BOUND('',#31,.T.);
#33 = ADVANCED_FACE('',(#32),#29,.T.);
#34 = AXIS2_PLACEMENT_3D('',#1,#2,$);
#35 = PLANE('',#34);
#36 = ORIENTED_EDGE('',*,*,#14,.F.);
#37 = EDGE_LOOP('',(#36));
#38 = FACE_OUTER_BOUND('',#37,.T.);
#39 = ADVANCED_FACE('',(#38),#35,.F.);
#40 = CLOSED_SHELL('',(#27,#33,#39));
#41 = MANIFOLD_SOLID_BREP('band',#40);
)");
}

/// A tube, 2 across its bore and 4 outside, 2 long, written as Alibre writes
/// one: every loop a plain FACE_BOUND, each end's inner circle first; each
/// side a band between its rims, the bore's facing into it.
inline std::string alibreTube() {
    return stepFile(R"(#1 = CARTESIAN_POINT('',(0.,0.,0.));
#2 = CARTESIAN_POINT('',(0.,0.,2.));
#3 = DIRECTION('',(0.,0.,1.));
#4 = DIRECTION('',(1.,0.,0.));
#5 = AXIS2_PLACEMENT_3D('',#1,#3,#4);
#6 = AXIS2_PLACEMENT_3D('',#2,#3,#4);
#7 = CIRCLE('',#5,2.);
#8 = CIRCLE('',#6,2.);
#9 = CIRCLE('',#5,1.);
#10 = CIRCLE('',#6,1.);
#31 = CARTESIAN_POINT('',(2.,0.,0.));
#32 = CARTESIAN_POINT('',(2.,0.,2.));
#33 = CARTESIAN_POINT('',(1.,0.,0.));
#34 = CARTESIAN_POINT('',(1.,0.,2.));
#11 = VERTEX_POINT('',#31);
#12 = VERTEX_POINT('',#32);
#13 = VERTEX_POINT('',#33);
#14 = VERTEX_POINT('',#34);
#15 = EDGE_CURVE('',#11,#11,#7,.T.);
#16 = EDGE_CURVE('',#12,#12,#8,.T.);
#17 = EDGE_CURVE('',#13,#13,#9,.T.);
#18 = EDGE_CURVE('',#14,#14,#10,.T.);
#40 = ORIENTED_EDGE('',*,*,#15,.T.);
#41 = ORIENTED_EDGE('',*,*,#16,.F.);
#42 = ORIENTED_EDGE('',*,*,#17,.F.);
#43 = ORIENTED_EDGE('',*,*,#18,.T.);
#44 = ORIENTED_EDGE('',*,*,#18,.F.);
#45 = ORIENTED_EDGE('',*,*,#16,.T.);
#46 = ORIENTED_EDGE('',*,*,#17,.T.);
#47 = ORIENTED_EDGE('',*,*,#15,.F.);
#50 = EDGE_LOOP('',(#40));
#51 = EDGE_LOOP('',(#41));
#52 = EDGE_LOOP('',(#42));
#53 = EDGE_LOOP('',(#43));
#54 = EDGE_LOOP('',(#44));
#55 = EDGE_LOOP('',(#45));
#56 = EDGE_LOOP('',(#46));
#57 = EDGE_LOOP('',(#47));
#60 = FACE_BOUND('',#50,.T.);
#61 = FACE_BOUND('',#51,.T.);
#62 = FACE_BOUND('',#52,.T.);
#63 = FACE_BOUND('',#53,.T.);
#64 = FACE_BOUND('',#54,.T.);
#65 = FACE_BOUND('',#55,.T.);
#66 = FACE_BOUND('',#56,.T.);
#67 = FACE_BOUND('',#57,.T.);
#70 = CYLINDRICAL_SURFACE('',#5,2.);
#71 = CYLINDRICAL_SURFACE('',#5,1.);
#72 = PLANE('',#6);
#73 = PLANE('',#5);
#20 = ADVANCED_FACE('',(#60,#61),#70,.T.);
#21 = ADVANCED_FACE('',(#62,#63),#71,.F.);
#22 = ADVANCED_FACE('',(#64,#65),#72,.T.);
#23 = ADVANCED_FACE('',(#66,#67),#73,.F.);
#24 = CLOSED_SHELL('',(#20,#21,#22,#23));
#25 = MANIFOLD_SOLID_BREP('tube',#24);
)");
}

/// A real as Part-21 writes one: always with its point.
inline std::string real(double v) {
    std::ostringstream out;
    out << std::setprecision(17) << v;
    std::string s = out.str();
    if (s.find_first_of(".eE") == std::string::npos) s += ".";
    return s;
}

/// A band (a cylinder of radius 1 and height 2 whose side is bounded by its two rims alone) whose
/// top rim, a circle of radius 1, is tilted @p tilt off the cylinder's parallels, its axis leaning
/// towards the angle @p lean about z, and its cap's plane with it. The rim's vertex is at @p at on
/// its circle, the bottom rim's at angle 0, so the band's seam splits the top rim.
inline std::string bandWithATiltedRim(double tilt, double lean, double at) {
    const math::Vec3 axis{std::sin(tilt) * std::cos(lean), std::sin(tilt) * std::sin(lean),
                          std::cos(tilt)};
    math::Vec3 x = math::Vec3{1, 0, 0} - axis * axis.x;
    x = x * (1.0 / x.length());
    const math::Vec3 vertex = math::Vec3{0, 0, 2} + x * std::cos(at) + axis.cross(x) * std::sin(at);
    const auto triple = [](const math::Vec3& v) {
        return "(" + real(v.x) + "," + real(v.y) + "," + real(v.z) + ")";
    };
    return "ISO-10303-21;\nHEADER;\nFILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\n"
           "#1 = CARTESIAN_POINT('',(0.,0.,0.));\n#2 = DIRECTION('',(0.,0.,1.));\n"
           "#3 = DIRECTION('',(0.,1.,0.));\n#4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);\n"
           "#5 = CIRCLE('',#4,1.);\n#6 = CARTESIAN_POINT('',(1.,0.,0.));\n"
           "#7 = VERTEX_POINT('',#6);\n#8 = CARTESIAN_POINT('',(0.,0.,2.));\n"
           "#9 = DIRECTION('',(1.,0.,0.));\n#50 = DIRECTION(''," +
           triple(axis) + ");\n#51 = DIRECTION(''," + triple(x) +
           ");\n#10 = AXIS2_PLACEMENT_3D('',#8,#50,#51);\n#11 = CIRCLE('',#10,1.);\n"
           "#12 = CARTESIAN_POINT(''," +
           triple(vertex) +
           ");\n#13 = VERTEX_POINT('',#12);\n#14 = EDGE_CURVE('',#7,#7,#5,.T.);\n"
           "#15 = EDGE_CURVE('',#13,#13,#11,.T.);\n#19 = AXIS2_PLACEMENT_3D('',#1,#2,#9);\n"
           "#20 = CYLINDRICAL_SURFACE('',#19,1.);\n#21 = ORIENTED_EDGE('',*,*,#14,.T.);\n"
           "#22 = EDGE_LOOP('',(#21));\n#23 = FACE_OUTER_BOUND('',#22,.T.);\n"
           "#24 = ORIENTED_EDGE('',*,*,#15,.F.);\n#25 = EDGE_LOOP('',(#24));\n"
           "#26 = FACE_BOUND('',#25,.T.);\n#27 = ADVANCED_FACE('',(#23,#26),#20,.T.);\n"
           "#28 = AXIS2_PLACEMENT_3D('',#8,#50,$);\n#29 = PLANE('',#28);\n"
           "#30 = ORIENTED_EDGE('',*,*,#15,.T.);\n#31 = EDGE_LOOP('',(#30));\n"
           "#32 = FACE_OUTER_BOUND('',#31,.T.);\n#33 = ADVANCED_FACE('',(#32),#29,.T.);\n"
           "#34 = AXIS2_PLACEMENT_3D('',#1,#2,$);\n#35 = PLANE('',#34);\n"
           "#36 = ORIENTED_EDGE('',*,*,#14,.F.);\n#37 = EDGE_LOOP('',(#36));\n"
           "#38 = FACE_OUTER_BOUND('',#37,.T.);\n#39 = ADVANCED_FACE('',(#38),#35,.F.);\n"
           "#40 = CLOSED_SHELL('',(#27,#33,#39));\n#41 = MANIFOLD_SOLID_BREP('band',#40);\n"
           "ENDSEC;\nEND-ISO-10303-21;\n";
}

/// A cap of a sphere of radius 10 about the origin as OpenCASCADE writes one: the sphere's face
/// north of the plane z = @p height, bounded by its rim and a seam meridian from the rim up to the
/// pole, used once each way, in the loop order @p order (a rotation of "#20,#21,#22"), and a flat
/// disc. With @p surface, the cap's surface is that entity text instead (it must define #5).
inline std::string seamedCap(double height, const std::string& order = "#20,#21,#22",
                             const std::string& surface = {}) {
    const double r = std::sqrt(100.0 - height * height);
    return stepFile(R"(#1 = CARTESIAN_POINT('',(0.,0.,0.));
#2 = DIRECTION('',(0.,0.,1.));
#3 = DIRECTION('',(1.,0.,0.));
#4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);
)" + (surface.empty() ? std::string("#5 = SPHERICAL_SURFACE('',#4,10.);\n") : surface) +
                    "#32 = CARTESIAN_POINT('',(0.,0.," + real(height) +
                    "));\n#33 = AXIS2_PLACEMENT_3D('',#32,#2,#3);\n#6 = CIRCLE('',#33," + real(r) +
                    ");\n#7 = CARTESIAN_POINT('',(" + real(r) + ",0.," + real(height) + R"());
#8 = VERTEX_POINT('',#7);
#9 = CARTESIAN_POINT('',(0.,0.,10.));
#10 = VERTEX_POINT('',#9);
#11 = DIRECTION('',(0.,-1.,0.));
#12 = AXIS2_PLACEMENT_3D('',#1,#11,#3);
#13 = CIRCLE('',#12,10.);
#14 = EDGE_CURVE('',#8,#8,#6,.T.);
#15 = EDGE_CURVE('',#8,#10,#13,.T.);
#16 = DIRECTION('',(0.,0.,-1.));
#17 = AXIS2_PLACEMENT_3D('',#32,#16,#3);
#18 = PLANE('',#17);
#20 = ORIENTED_EDGE('',*,*,#14,.T.);
#21 = ORIENTED_EDGE('',*,*,#15,.T.);
#22 = ORIENTED_EDGE('',*,*,#15,.F.);
#23 = EDGE_LOOP('',()" +
                    order +
                    R"());
#24 = FACE_OUTER_BOUND('',#23,.T.);
#25 = ADVANCED_FACE('',(#24),#5,.T.);
#26 = ORIENTED_EDGE('',*,*,#14,.F.);
#27 = EDGE_LOOP('',(#26));
#28 = FACE_OUTER_BOUND('',#27,.T.);
#29 = ADVANCED_FACE('',(#28),#18,.T.);
#30 = CLOSED_SHELL('',(#25,#29));
#31 = MANIFOLD_SOLID_BREP('cap',#30);
)");
}

/// seamedCap's hemisphere: its rim the equator, of radius 10.
inline std::string seamedHemisphere(const std::string& order = "#20,#21,#22",
                                    const std::string& surface = {}) {
    return seamedCap(0.0, order, surface);
}

/// A sphere of radius @p radius about the origin as a rational B-spline, as other systems write
/// one: entity #5, biquadratic, nine control points round z (from +x) by five from the south pole
/// to the north (control points #200 on). Its first and last rows of five each collapse to a
/// pole. u runs over [0, 1] round z, v over [0, 1] from pole to pole. With @p northSpread, the
/// north pole's row comes together only within it: its seven points off the seam move by it, in
/// turn up and down the axis (or, with @p alongAxis false, either way along x).
inline std::string splineSphereSurface(double radius, double northSpread = 0.0,
                                       bool alongAxis = true) {
    const double w = std::sqrt(0.5);
    const double xs[9] = {1, 1, 0, -1, -1, -1, 0, 1, 1};
    const double ys[9] = {0, 1, 1, 1, 0, -1, -1, -1, 0};
    const double rs[5] = {0, 1, 1, 1, 0};
    const double zs[5] = {-1, -1, 0, 1, 1};
    const double wv[5] = {1, w, 1, w, 1};
    std::string text;
    std::string net;
    std::string weights;
    int id = 200;
    for (int i = 0; i < 9; ++i) {
        const double wu = i % 2 == 1 ? w : 1.0;
        net += i == 0 ? "(" : ",(";
        weights += i == 0 ? "(" : ",(";
        for (int j = 0; j < 5; ++j) {
            const double spread =
                j == 4 && i >= 1 && i <= 7 ? (i % 2 == 0 ? northSpread : -northSpread) : 0.0;
            // Only a row that moves is written differently: 0 added to -0 would write "0.".
            double x = radius * xs[i] * rs[j];
            double z = radius * zs[j];
            if (spread != 0.0) (alongAxis ? z : x) += spread;
            text += "#" + std::to_string(id) + " = CARTESIAN_POINT('',(" + real(x) + "," +
                    real(radius * ys[i] * rs[j]) + "," + real(z) + "));\n";
            net += std::string(j == 0 ? "#" : ",#") + std::to_string(id);
            weights += std::string(j == 0 ? "" : ",") + real(wu * wv[j]);
            ++id;
        }
        net += ")";
        weights += ")";
    }
    return text + "#5 = ( BOUNDED_SURFACE() B_SPLINE_SURFACE(2,2,(" + net +
           "),.UNSPECIFIED.,.T.,.F.,.F.) B_SPLINE_SURFACE_WITH_KNOTS((3,2,2,2,3),(3,2,3),"
           "(0.,0.25,0.5,0.75,1.),(0.,0.5,1.),.UNSPECIFIED.) GEOMETRIC_REPRESENTATION_ITEM() "
           "RATIONAL_B_SPLINE_SURFACE((" +
           weights + ")) REPRESENTATION_ITEM('') SURFACE() );\n";
}

/// externalStyleCylinder() with its side a rational B-spline: the same cylinder, a quadric
/// written as a B_SPLINE_SURFACE, from (1, 0) round about +z, as nine control points a row.
inline std::string splineCylinder() {
    const double w = std::sqrt(0.5);
    const double xs[9] = {1, 1, 0, -1, -1, -1, 0, 1, 1};
    const double ys[9] = {0, 1, 1, 1, 0, -1, -1, -1, 0};
    std::string points;
    std::string net;
    std::string weights;
    int id = 200;
    for (int i = 0; i < 9; ++i) {
        const double wi = i % 2 == 1 ? w : 1.0;
        net += std::string(i == 0 ? "(" : ",(");
        weights += std::string(i == 0 ? "(" : ",(") + real(wi) + "," + real(wi) + ")";
        for (int k = 0; k < 2; ++k) {
            points += "#" + std::to_string(id) + " = CARTESIAN_POINT('',(" + real(xs[i]) + "," +
                      real(ys[i]) + "," + real(2.0 * k) + "));\n";
            net += std::string(k == 0 ? "#" : ",#") + std::to_string(id);
            ++id;
        }
        net += ")";
    }
    const std::string surface =
        "#20 = ( BOUNDED_SURFACE() B_SPLINE_SURFACE(2,1,(" + net +
        "),.UNSPECIFIED.,.T.,.F.,.F.) B_SPLINE_SURFACE_WITH_KNOTS((3,2,2,2,3),(2,2),"
        "(0.,0.25,0.5,0.75,1.),(0.,1.),.UNSPECIFIED.) GEOMETRIC_REPRESENTATION_ITEM() "
        "RATIONAL_B_SPLINE_SURFACE((" +
        weights + ")) REPRESENTATION_ITEM('') SURFACE() );\n";
    std::string text = externalStyleCylinder();
    const std::string side = "#20 = CYLINDRICAL_SURFACE('',#19,1.);\n";
    text.replace(text.find(side), side.size(), points + surface);
    return text;
}

/// A Part-21 DATA section written an entity at a time.
struct Data {
    std::string text;
    int next = 1;
    int add(const std::string& entity) {
        text += "#" + std::to_string(next) + " = " + entity + ";\n";
        return next++;
    }
    static std::string ref(int id) { return "#" + std::to_string(id); }
    int point(const math::Vec3& p) {
        return add("CARTESIAN_POINT('',(" + real(p.x) + "," + real(p.y) + "," + real(p.z) + "))");
    }
    int direction(const math::Vec3& d) {
        return add("DIRECTION('',(" + real(d.x) + "," + real(d.y) + "," + real(d.z) + "))");
    }
    int placement(const math::Vec3& origin, const math::Vec3& z, const math::Vec3& x) {
        const int o = point(origin);
        const int zd = direction(z);
        const int xd = direction(x);
        return add("AXIS2_PLACEMENT_3D(''," + ref(o) + "," + ref(zd) + "," + ref(xd) + ")");
    }
    /// A face of @p loop, each of whose edges is (EDGE_CURVE, along it), on @p surface.
    int face(const std::vector<std::pair<int, bool>>& loop, int surface, bool sameSense = true) {
        std::string list;
        for (const auto& [edge, along] : loop) {
            const int oe =
                add("ORIENTED_EDGE('',*,*," + ref(edge) + "," + (along ? ".T." : ".F.") + ")");
            list += (list.empty() ? "" : ",") + ref(oe);
        }
        const int edgeLoop = add("EDGE_LOOP('',(" + list + "))");
        const int bound = add("FACE_OUTER_BOUND(''," + ref(edgeLoop) + ",.T.)");
        return add("ADVANCED_FACE('',(" + ref(bound) + ")," + ref(surface) + "," +
                   (sameSense ? ".T." : ".F.") + ")");
    }
};

/// The closed shell of a block from (0, 0, 0) to (4, 4, 4) into @p d: six PLANE faces, each loop
/// counter-clockwise seen from outside. With @p frontFacingIn, the front face's (y = 0) plane
/// faces into the block while its face still says same_sense .T., as Horizon's own writer wrote
/// faces whichever way their surfaces were turned. Its CLOSED_SHELL's id.
inline int blockShell(Data& d, bool frontFacingIn = false) {
    const math::Vec3 corner[8] = {{0, 0, 0}, {4, 0, 0}, {4, 4, 0}, {0, 4, 0},
                                  {0, 0, 4}, {4, 0, 4}, {4, 4, 4}, {0, 4, 4}};
    int points[8];
    int vertices[8];
    for (int i = 0; i < 8; ++i) {
        points[i] = d.point(corner[i]);
        vertices[i] = d.add("VERTEX_POINT(''," + Data::ref(points[i]) + ")");
    }
    const auto edge = [&](int a, int b) {
        const int dir = d.direction((corner[b] - corner[a]).normalized());
        const int vec = d.add("VECTOR(''," + Data::ref(dir) + ",1.)");
        const int line = d.add("LINE(''," + Data::ref(points[a]) + "," + Data::ref(vec) + ")");
        return d.add("EDGE_CURVE(''," + Data::ref(vertices[a]) + "," + Data::ref(vertices[b]) +
                     "," + Data::ref(line) + ",.T.)");
    };
    const int e1 = edge(0, 1);
    const int e2 = edge(1, 2);
    const int e3 = edge(2, 3);
    const int e4 = edge(3, 0);
    const int e5 = edge(4, 5);
    const int e6 = edge(5, 6);
    const int e7 = edge(6, 7);
    const int e8 = edge(7, 4);
    const int e9 = edge(0, 4);
    const int e10 = edge(1, 5);
    const int e11 = edge(2, 6);
    const int e12 = edge(3, 7);
    const auto plane = [&](const math::Vec3& origin, const math::Vec3& normal,
                           const math::Vec3& x) {
        return d.add("PLANE(''," + Data::ref(d.placement(origin, normal, x)) + ")");
    };
    std::vector<int> faces;
    // Each loop counter-clockwise seen from outside.
    faces.push_back(d.face({{e4, false}, {e3, false}, {e2, false}, {e1, false}},
                           plane({0, 0, 0}, {0, 0, -1}, {1, 0, 0})));
    faces.push_back(d.face({{e5, true}, {e6, true}, {e7, true}, {e8, true}},
                           plane({0, 0, 4}, {0, 0, 1}, {1, 0, 0})));
    faces.push_back(d.face({{e1, true}, {e10, true}, {e5, false}, {e9, false}},
                           plane({0, 0, 0}, {0, frontFacingIn ? 1.0 : -1.0, 0}, {1, 0, 0})));
    faces.push_back(d.face({{e3, true}, {e12, true}, {e7, false}, {e11, false}},
                           plane({0, 4, 0}, {0, 1, 0}, {-1, 0, 0})));
    faces.push_back(d.face({{e9, true}, {e8, false}, {e12, false}, {e4, true}},
                           plane({0, 0, 0}, {-1, 0, 0}, {0, 1, 0})));
    faces.push_back(d.face({{e2, true}, {e11, true}, {e6, false}, {e10, false}},
                           plane({4, 0, 0}, {1, 0, 0}, {0, 1, 0})));
    std::string list;
    for (int f : faces) list += (list.empty() ? "" : ",") + Data::ref(f);
    return d.add("CLOSED_SHELL('',(" + list + "))");
}

/// A C-shaped clip: the ring between radii 1 and 2 about the z axis through @p centre, swept 300
/// degrees counter-clockwise from -150 to +150 degrees, from @p centre up to 1 above it. Six
/// faces: its top and bottom PLANEs, its two ends, the CYLINDRICAL_SURFACEs of its outside and of
/// its inside (that one's face against its surface), each loop counter-clockwise seen from
/// outside. Its arcs go more than half way round, so each flat face's corners wind against the
/// face, and its corners joined by straight lines enclose a negative volume: it faces out only
/// as its faces say. Volume 2.5 pi.
inline std::string cClip(const math::Vec3& centre = {}) {
    Data d;
    constexpr double kDegree = 3.14159265358979323846 / 180.0;
    const double start = -150.0 * kDegree;
    const double end = 150.0 * kDegree;
    struct Corner {
        math::Vec3 at;
        int point;
        int vertex;
    };
    const auto corner = [&](double radius, double angle, double z) {
        const math::Vec3 at =
            centre + math::Vec3(radius * std::cos(angle), radius * std::sin(angle), z);
        const int point = d.point(at);
        return Corner{at, point, d.add("VERTEX_POINT(''," + Data::ref(point) + ")")};
    };
    const Corner o1b = corner(2, start, 0);
    const Corner o2b = corner(2, end, 0);
    const Corner i1b = corner(1, start, 0);
    const Corner i2b = corner(1, end, 0);
    const Corner o1t = corner(2, start, 1);
    const Corner o2t = corner(2, end, 1);
    const Corner i1t = corner(1, start, 1);
    const Corner i2t = corner(1, end, 1);
    const auto line = [&](const Corner& a, const Corner& b) {
        const int dir = d.direction((b.at - a.at).normalized());
        const int vec = d.add("VECTOR(''," + Data::ref(dir) + ",1.)");
        const int curve = d.add("LINE(''," + Data::ref(a.point) + "," + Data::ref(vec) + ")");
        return d.add("EDGE_CURVE(''," + Data::ref(a.vertex) + "," + Data::ref(b.vertex) + "," +
                     Data::ref(curve) + ",.T.)");
    };
    // Each arc counter-clockwise about +z, 300 degrees from its start to its end.
    const auto arc = [&](const Corner& a, const Corner& b, double radius, double z) {
        const int circle =
            d.add("CIRCLE(''," +
                  Data::ref(d.placement(centre + math::Vec3(0, 0, z), {0, 0, 1}, {1, 0, 0})) + "," +
                  real(radius) + ")");
        return d.add("EDGE_CURVE(''," + Data::ref(a.vertex) + "," + Data::ref(b.vertex) + "," +
                     Data::ref(circle) + ",.T.)");
    };
    const int outerBottom = arc(o1b, o2b, 2, 0);
    const int outerTop = arc(o1t, o2t, 2, 1);
    const int innerBottom = arc(i1b, i2b, 1, 0);
    const int innerTop = arc(i1t, i2t, 1, 1);
    const int upOuter1 = line(o1b, o1t);
    const int upOuter2 = line(o2b, o2t);
    const int upInner1 = line(i1b, i1t);
    const int upInner2 = line(i2b, i2t);
    const int outBottom1 = line(i1b, o1b);
    const int outBottom2 = line(i2b, o2b);
    const int outTop1 = line(i1t, o1t);
    const int outTop2 = line(i2t, o2t);
    const auto plane = [&](const math::Vec3& origin, const math::Vec3& normal,
                           const math::Vec3& x) {
        return d.add("PLANE(''," + Data::ref(d.placement(origin, normal, x)) + ")");
    };
    const auto cylinder = [&](double radius) {
        return d.add("CYLINDRICAL_SURFACE(''," +
                     Data::ref(d.placement(centre, {0, 0, 1}, {1, 0, 0})) + "," + real(radius) +
                     ")");
    };
    const math::Vec3 radial1(std::cos(start), std::sin(start), 0);
    const math::Vec3 radial2(std::cos(end), std::sin(end), 0);
    std::vector<int> faces;
    faces.push_back(d.face({{outerTop, true}, {outTop2, false}, {innerTop, false}, {outTop1, true}},
                           plane(centre + math::Vec3(0, 0, 1), {0, 0, 1}, {1, 0, 0})));
    faces.push_back(
        d.face({{outBottom1, false}, {innerBottom, true}, {outBottom2, true}, {outerBottom, false}},
               plane(centre, {0, 0, -1}, {1, 0, 0})));
    faces.push_back(
        d.face({{outerBottom, true}, {upOuter2, true}, {outerTop, false}, {upOuter1, false}},
               cylinder(2)));
    faces.push_back(
        d.face({{innerBottom, false}, {upInner1, true}, {innerTop, true}, {upInner2, false}},
               cylinder(1), false));
    // Each end faces away from the ring: back along it at its start, on along it at its end.
    faces.push_back(
        d.face({{outBottom1, true}, {upOuter1, true}, {outTop1, false}, {upInner1, false}},
               plane(centre, {std::sin(start), -std::cos(start), 0}, radial1)));
    faces.push_back(
        d.face({{outBottom2, false}, {upInner2, true}, {outTop2, true}, {upOuter2, false}},
               plane(centre, {-std::sin(end), std::cos(end), 0}, radial2)));
    std::string list;
    for (int f : faces) list += (list.empty() ? "" : ",") + Data::ref(f);
    const int shell = d.add("CLOSED_SHELL('',(" + list + "))");
    d.add("MANIFOLD_SOLID_BREP('clip'," + Data::ref(shell) + ")");
    return stepFile(d.text);
}

/// blockShell()'s block alone, as one solid.
inline std::string block(bool frontFacingIn) {
    Data d;
    d.add("MANIFOLD_SOLID_BREP('block'," + Data::ref(blockShell(d, frontFacingIn)) + ")");
    return stepFile(d.text);
}

/// The block with a ball of radius 1 cut out of its middle: its void a sphere whose one face is
/// bounded by a seam from pole to pole and back, as OpenCASCADE writes one, the shell facing out
/// of the void and turned round by its ORIENTED_CLOSED_SHELL, as the standard has every void.
inline std::string blockWithABall() {
    Data d;
    const int outer = blockShell(d);

    // The ball: a meridian from its south pole to its north, on its +x side.
    const math::Vec3 centre{2, 2, 2};
    const int ball = d.add("SPHERICAL_SURFACE(''," +
                           Data::ref(d.placement(centre, {0, 0, 1}, {1, 0, 0})) + ",1.)");
    const int south = d.add("VERTEX_POINT(''," + Data::ref(d.point({2, 2, 1})) + ")");
    const int north = d.add("VERTEX_POINT(''," + Data::ref(d.point({2, 2, 3})) + ")");
    const int meridian =
        d.add("CIRCLE(''," + Data::ref(d.placement(centre, {0, -1, 0}, {1, 0, 0})) + ",1.)");
    const int seam = d.add("EDGE_CURVE(''," + Data::ref(south) + "," + Data::ref(north) + "," +
                           Data::ref(meridian) + ",.T.)");
    const int skin = d.face({{seam, true}, {seam, false}}, ball);
    const int voidShell = d.add("CLOSED_SHELL('',(" + Data::ref(skin) + "))");
    const int inward = d.add("ORIENTED_CLOSED_SHELL('',*," + Data::ref(voidShell) + ",.F.)");
    d.add("BREP_WITH_VOIDS('block'," + Data::ref(outer) + ",(" + Data::ref(inward) + "))");
    return stepFile(d.text);
}

}  // namespace hz::step_text
