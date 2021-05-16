#include "../barycentric_svd/barycentric_svd.h"
#include "test_case.h"
#include <survive_reproject.h>

void fill_m(void *user, FLT *eq, int axis, FLT angle) {
	FLT sv = sin(angle), cv = cos(angle);
	switch (axis) {
	case 0:
		eq[0] = cv;
		eq[1] = 0;
		eq[2] = -sv;
		break;
	case 1:
		eq[0] = 0;
		eq[1] = cv;
		eq[2] = -sv;
		break;
	}
}

TEST(BarycentricSVD, Basic) {
	LinmathPoint3d pts[] = {{
								1,
								0,
								0,
							},
							{
								0,
								1,
								0,
							},
							{0, 0, 1},
							{
								1,
								0,
								1,
							},
							{
								0,
								1,
								1,
							},
							{0, 0, 0},
							{
								1,
								1,
								1,
							},
							{
								1,
								1,
								0,
							},
							{1, 0, 1}};

	LinmathPoint2d meas[sizeof(pts) / sizeof(pts[0])] = {0};

	BaseStationCal bsd_cal[2] = {0};
	SurvivePose pose = {.Pos = {.25, -.33, -2.}, .Rot = {1, .1, .2, .3}};
	SurvivePose ipose = InvertPoseRtn(&pose);
	quatnormalize(pose.Rot, pose.Rot);

	for (int i = 0; i < sizeof(pts) / sizeof(pts[0]); i++) {
		LinmathPoint3d ptInLH = {0};
		ApplyPoseToPoint(ptInLH, &pose, pts[i]);
		survive_reproject_xy(bsd_cal, ptInLH, (FLT *)&meas[i]);
	}

	bc_svd bc = {0};
	bc_svd_bc_svd(&bc, 0, fill_m, pts, sizeof(pts) / sizeof(pts[0]));

	bc_svd_reset_correspondences(&bc);

	for (int i = 0; i < sizeof(pts) / sizeof(pts[0]); i++) {
		for (int axis = 0; axis < 2; axis++) {
			bc_svd_add_single_correspondence(&bc, i, axis, meas[i][axis]);
		}
	}

	FLT R[3][3];

	SurvivePose output = {0};
	FLT svd_err = bc_svd_compute_pose(&bc, R, output.Pos);

	FLT control_pts[4][3] = {{-0.68469785575048625, 0.30937621832358714, 1.5341130604288495},
							 {-1.2084806566294102, 0.20461965814780203, 1.4293565002530646},
							 {-0.5734401517586627, 0.19723720947413526, 1.089963549319187},
							 {-0.62080240479924753, -0.1394184031254535, 1.6634304271216975}};

	FLT diff[12] = {0};
	FLT err = 0;
	subnd(diff, (const FLT *)bc.control_points_in_camera, (const FLT *)control_pts, 12);
	for (int i = 0; i < 12; i++)
		err += diff[i];
	// assert(err <= 1e-5);

	LinmathQuat tmp;
	quatfrommatrix33(tmp, (const FLT *)&R);

	// Typical camera applications have Z facing forward; the vive is contrarian and has Z going out of the
	// back of the lighthouse. Think of this as a rotation on the Y axis a full 180 degrees -- the quat for that is
	// [0 0x 1y 0z]
	const LinmathQuat rt = {0, 0, 1, 0};
	LinmathQuat output_rot;

	quatrotateabout(output.Rot, tmp, rt);

	{ // if (!cameraToWorld) {
		// We have to pre-multiply the rt transform here, which means we have to also offset our position by
		quatrotateabout(output.Rot, rt, tmp);
		output.Pos[0] = -output.Pos[0];
		output.Pos[2] = -output.Pos[2];
	}

	if (output.Rot[0] < 0)
		scalend(output.Rot, output.Rot, -1, 4);

	FLT *a = (FLT *)output.Pos;
	FLT *b = (FLT *)pose.Pos;
	for (int i = 0; i < 7; i++) {
		assert(fabs(a[i] - b[i]) < .0001);
	}

	bc_svd_dtor(&bc);
	return 0;
}
