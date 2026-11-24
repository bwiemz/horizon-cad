#pragma once

// STEP text for the reader's tests, written by hand as other systems write their solids.
// Shared by test_StepFormat, test_StepCurved and test_StepExact.

#include <string>

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

}  // namespace hz::step_text
