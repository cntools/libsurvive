
#include "linmath.h"
#ifndef WIN32
#include <alloca.h>
#endif
#include <assert.h>
#if !defined(__FreeBSD__) && !defined(__APPLE__)
#include <malloc.h>
#endif
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

bool assertFLTEquals(FLT a, FLT b) { return fabs(a - b) < 0.0001; }

int assertFLTAEquals(FLT *a, FLT *b, int length) {
	for (int i = 0; i < length; i++) {
		if (assertFLTEquals(a[i], b[i]) != true) {
			return i;
		}
	}
	return -1;
}

void printFLTA(FLT *a, int length) {
	for (int i = 0; i < length; i++)
		fprintf(stderr, "%.7f ", a[i]);
}

#define ASSERT_FLTA_EQUALS(a, b, l)                                                                                    \
	if (assertFLTAEquals(a, b, l) != -1) {                                                                             \
		fprintf(stderr, "Where '" #a "'= ");                                                                           \
		printFLTA(a, l);                                                                                               \
		fprintf(stderr, "\nWhere '" #b "'= ");                                                                         \
		printFLTA(b, l);                                                                                               \
		fprintf(stderr, "\n");                                                                                         \
		assert(assertFLTAEquals(a, b, l) == -1);                                                                       \
	}

void testInvertPose() {

	LinmathPose rotAroundYOffset = {1, 1, 1, .5, 0, .5, 0};
	LinmathPose pose_out;

	{
		LinmathPose expected = {1, -1, -1, 0.7071068, 0, -0.7071068, 0};
		InvertPose(&pose_out, &rotAroundYOffset);
		ASSERT_FLTA_EQUALS(pose_out.Pos, expected.Pos, 7);

		FLT identity[] = {0, 0, 0, 1, 0, 0, 0};
		ApplyPoseToPose(&pose_out, &expected, &rotAroundYOffset);
		quatnormalize(pose_out.Rot, pose_out.Rot);
		ASSERT_FLTA_EQUALS(pose_out.Pos, identity, 7);
	}
}

void testApplyPoseToPose() {
	LinmathPose rotAroundYOffset = {1, 1, 1, 0, 0, 1, 0};
	LinmathPose pose_out;

	{
		LinmathPose pt = {0, 1, 0, 0, 0, 1, 0};
		LinmathPose expected = {1, 2, 1, -1, 0, 0, 0};
		ApplyPoseToPose(&pose_out, &rotAroundYOffset, &pt);
		quatnormalize(pose_out.Rot, pose_out.Rot);
		ASSERT_FLTA_EQUALS(pose_out.Pos, expected.Pos, 7);
	}

	{
		LinmathPose pt = {0, 1, 0, 0, 1, 0, 0};
		LinmathPose expected = {1, 2, 1, 0, 0, 0, -1};
		ApplyPoseToPose(&pose_out, &rotAroundYOffset, &pt);
		ASSERT_FLTA_EQUALS(pose_out.Pos, expected.Pos, 7);
	}

	{
	  pose_out = rotAroundYOffset;
	  LinmathPose expected = {0, 2, 0, -1, 0, 0, 0};
	  ApplyPoseToPose(&pose_out, &pose_out, &pose_out);
	  ASSERT_FLTA_EQUALS(pose_out.Pos, expected.Pos, 7);
	}

}

void testApplyPoseToPoint() {
	LinmathPose rotAroundY = {0, 0, 0, 0, 0, 1, 0};
	FLT pt_out[3];

	{
		FLT pt[3] = {0, 1, 0};
		FLT expected[3] = {0, 1, 0};
		ApplyPoseToPoint(pt_out, &rotAroundY, pt);
		ASSERT_FLTA_EQUALS(pt_out, expected, 3);
	}

	{
		FLT pt[3] = {1, 1, 0};
		FLT expected[3] = {-1, 1, 0};
		ApplyPoseToPoint(pt_out, &rotAroundY, pt);
		ASSERT_FLTA_EQUALS(pt_out, expected, 3);
	}

	LinmathPose rotAroundYOffset = {1, 1, 1, 0, 0, 1, 0};

	{
		FLT pt[3] = {0, 1, 0};
		FLT expected[3] = {1, 2, 1};
		ApplyPoseToPoint(pt_out, &rotAroundYOffset, pt);
		ASSERT_FLTA_EQUALS(pt_out, expected, 3);
	}

	{
		FLT pt[3] = {1, 1, 0};
		FLT expected[3] = {0, 2, 1};
		ApplyPoseToPoint(pt_out, &rotAroundYOffset, pt);
		ASSERT_FLTA_EQUALS(pt_out, expected, 3);
	}
}

void testKabsch() {
	FLT pts[] = {0, 0, 0, 100, 100, 100, 10, 0, 10, 50, 50, 0, 0, 0, 1000, -100, 0, 100};

	LinmathPose pts2txPts = {.Pos = { 0 }, .Rot = {4, 3, 2, 1}};

	LinmathPose pts2tx2Pts = {.Pos = {1, 2, 3}, .Rot = {1, 2, 3, 4}};

	quatnormalize(pts2txPts.Rot, pts2txPts.Rot);
	quatnormalize(pts2tx2Pts.Rot, pts2tx2Pts.Rot);

	const int N = sizeof(pts) / sizeof(FLT) / 3;

	FLT *txPts = alloca(N * 3 * sizeof(FLT));
	FLT *txPts2 = alloca(N * 3 * sizeof(FLT));
	for (int i = 0; i < N; i++) {
		ApplyPoseToPoint(txPts + i * 3, &pts2txPts, pts + i * 3);
		ApplyPoseToPoint(txPts2 + i * 3, &pts2tx2Pts, pts + i * 3);
	}

	LinmathQuat should_be_tx = { 0 };
	KabschCentered(should_be_tx, pts, txPts, N);
	ASSERT_FLTA_EQUALS(should_be_tx, pts2txPts.Rot, 4);

	LinmathPose should_be_tx2 = { 0 };
	Kabsch(&should_be_tx2, pts, txPts2, N);
	ASSERT_FLTA_EQUALS(should_be_tx2.Pos, pts2tx2Pts.Pos, 7);
}

static void testKabsch2() {
	LinmathQuat q = {0};

	LinmathPoint3d survivePts[] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

	LinmathPoint3d openvrPts[] = {{1, 0, 0}, {0, 0, 1}, {0, 1, 0}};

	KabschCentered(q, (FLT *)survivePts, (FLT *)openvrPts, 3);
}

static void testQuatFind(const LinmathQuat _q1, const LinmathQuat _q2) {
	LinmathQuat q1;
	quatnormalize(q1, _q1);
	LinmathQuat q2;
	quatnormalize(q2, _q2);

	LinmathQuat q;
	quatfind(q, q1, q2);
	LinmathQuat tx_q2;
	quatrotateabout(tx_q2, q, q1);

	ASSERT_FLTA_EQUALS(tx_q2, q2, 4);
}
static void testQuatRotate() {
	LinmathPoint3d out;
	LinmathPoint3d out_gt = {3.0666666666666664, -0.66666666666666607, -1.4666666666666659};

	LinmathPoint3d in = {2, 2, 2};
	LinmathQuat q = {.1, .5, .2, 0};
	quatnormalize(q, q);

	quatrotatevector(out, q, in);
	ASSERT_FLTA_EQUALS(out, out_gt, 3);
}
static void testQuatFind_between_vectors(const LinmathPoint3d _tmpA, const LinmathPoint3d _tmpB) {
	LinmathQuat q;
	LinmathPoint3d tmpA, tmpB;

	normalize3d(tmpA, _tmpA);
	normalize3d(tmpB, _tmpB);

	quatfind_between_vectors(q, tmpA, tmpB);

	LinmathPoint3d out;
	quatrotatevector(out, q, tmpA);
	assert(dist3d(tmpB, out) < .00001);
}

static void testQuatFinding() {
	{ testQuatFind(LinmathQuat_Identity, LinmathQuat_Identity); }
	{
		LinmathQuat q2 = {0.7071068, 0, 0, 0.7071068};
		testQuatFind(LinmathQuat_Identity, q2);
	}
	{
		LinmathQuat q1 = {1., .3, .1, -.4};
		LinmathQuat q2 = {.2, -.13, .1, -.4};
		testQuatFind(q1, q2);
	}

	{
		LinmathPoint3d q1 = {1., .3, .1};
		LinmathPoint3d q2 = {.2, -.13, .1};

		testQuatFind_between_vectors(q1, q2);
	}
	{
		LinmathPoint3d q1 = {1, 1, 1};
		LinmathPoint3d q2 = {1, 1, 1};
		testQuatFind_between_vectors(q1, q2);
	}
	{
		LinmathPoint3d q1 = {1, 1, .999};
		LinmathPoint3d q2 = {1, 1, 1};
		testQuatFind_between_vectors(q1, q2);
	}
	{
		LinmathPoint3d q1 = {-1, -1, -1};
		LinmathPoint3d q2 = {1, 1, 1};
		testQuatFind_between_vectors(q1, q2);
	}
	{
		LinmathPoint3d q1 = {-1, 1, 1};
		LinmathPoint3d q2 = {1, 1, 1};
		testQuatFind_between_vectors(q1, q2);
	}
	{
		LinmathPoint3d q1 = {0, .001, 0};
		LinmathPoint3d q2 = {1, 1, 1};
		testQuatFind_between_vectors(q1, q2);
	}
}

static void testQuatAsAngularVelocity() {
	{
		LinmathQuat q;
		quatmultiplyrotation(q, LinmathQuat_Identity, 1);
		ASSERT_FLTA_EQUALS(q, LinmathQuat_Identity, 4);
	}

	{
		LinmathQuat q = {0.9998766, 0, 0, 0.0157073};
		quatmultiplyrotation(q, q, 100);
		LinmathQuat qz = {0, 0, 0, 1};
		ASSERT_FLTA_EQUALS(q, qz, 4);
	}
}

static void testFindBestIntersections() {
	size_t num = 10;
	struct LinmathLine3d lines[10] = {0};

	LinmathPoint3d gt_pt = {linmath_rand(-10, 10), linmath_rand(-10, 10), linmath_rand(-10, 10)};
	FLT sigmas[] = {0, .01, .1, 1.};

	for (int s = 0; s < sizeof(sigmas) / sizeof(sigmas[0]); s++) {
		FLT sigma = sigmas[s];

		for (int i = 0; i < num; i++) {
			LinmathPoint3d noise = {linmath_normrand(0, sigma), linmath_normrand(0, sigma), linmath_normrand(0, sigma)};
			LinmathPoint3d pt;
			add3d(pt, gt_pt, noise);

			LinmathPoint3d dir = {linmath_rand(-1, 1), linmath_rand(-1, 1), linmath_rand(-1, 1)};
			normalize3d(dir, dir);

			FLT t1 = linmath_rand(-10, 10);
			FLT t2 = linmath_rand(-10, 10);

			struct LinmathLine3d *line = &lines[i];
			scale3d(line->a, dir, t1);
			add3d(line->a, line->a, pt);

			scale3d(line->b, dir, t2);
			add3d(line->b, line->b, pt);
		}

		LinmathPoint3d o_pt = {0};
		linmath_find_best_intersection(o_pt, lines, num);

		FLT err = dist3d(gt_pt, o_pt);
		assert(err < ( sqrt(3 * sigma * sigma) + 1e-5));
	}
}
static void testNormPdf() { assertFLTEquals(linmath_norm_pdf(-2, -1.1, 1.34), 0.23760171); }

static LinmathPose randomPose() {
	LinmathPose out;
	for (int i = 0; i < 3; i++)
		out.Pos[i] = linmath_rand(-10, 10);
	for (int i = 0; i < 4; i++)
		out.Rot[i] = linmath_rand(-10, 10);
	quatnormalize(out.Rot, out.Rot);
	return out;
}
static LinmathAxisAnglePose randomAAPose() {
	LinmathAxisAnglePose out;
	for (int i = 0; i < 3; i++)
		out.Pos[i] = linmath_rand(-10, 10);
	for (int i = 0; i < 3; i++)
		out.AxisAngleRot[i] = linmath_rand(-2, 2);
	return out;
}

static void testAxisAnglePoses() {
	LinmathPose p1 = randomPose();
	LinmathAxisAnglePose aa_p1 = Pose2AAPose(&p1);
	LinmathPose ip1 = InvertPoseRtn(&p1);
	LinmathAxisAnglePose aa_ip1 = Pose2AAPose(&ip1);
	LinmathAxisAnglePose aa_ip1p = InvertAAPoseRtn(&aa_p1);
	ASSERT_FLTA_EQUALS(aa_ip1.Pos, aa_ip1p.Pos, 6);

	LinmathAxisAngleVelocity v = randomAAPose(), v1, v2;

	scalend(v1.Pos, v.Pos, 10, 6);
	scalend(v2.Pos, v.Pos, -10, 6);
	LinmathAxisAnglePose iv1 = InvertAAPoseRtn(&v1);
	LinmathAxisAnglePose iv2 = InvertAAPoseRtn(&v2);

	LinmathPose o2w = randomPose();
	LinmathPose w2o = InvertPoseRtn(&o2w);

	LinmathPose p2o = AAPose2Pose(&v1);
	LinmathPose p2w;
	ApplyPoseToPose(&p2w, &o2w, &p2o);
	LinmathPose w2p = InvertPoseRtn(&p2w);

	LinmathPose o2p = InvertPoseRtn(&p2o);
	LinmathPose w2p_;
	ApplyPoseToPose(&w2p_, &o2p, &w2o);
}

int main()
{
	testNormPdf();
	testQuatRotate();
	testFindBestIntersections();

	testInvertPose();
	testApplyPoseToPoint();
	testApplyPoseToPose();
	testKabsch();
	testKabsch2();

	testQuatFinding();
	testQuatAsAngularVelocity();

	testAxisAnglePoses();
#if 1

#define NONTRANSPOSED_DAVE
#ifdef NONTRANSPOSED_DAVE
	FLT pLH1[3] = {-0.396888, 3.182945, -0.568622};
	FLT qLH1[4] = {0.668640, -0.576296, 0.103727, -0.458305};
	FLT pNLH1[3] = { 0.113572, 2.791495, -1.495652 };  //1M +x
	FLT qNLH1[4] = { 0.807419, 0.372818, -0.451339, 0.073308 };


	FLT pLH2[3] = {0.195579, 3.193770, -0.424473};
	FLT qLH2[4] = {0.401849, 0.104771, 0.580441, 0.700449};
	FLT pNLH2[3] = {-0.183505, 3.356293, 0.695688, };
	FLT qNLH2[4] = {-0.237438, 0.405213, 0.270438, 0.840410 };
#else

	FLT pLH1[3] = {-0.321299, 3.130532, -0.786460};
	FLT qLH1[4] = {0.794180, 0.336117, -0.485668, -0.142934};
	FLT pNLH1[3] = { 0.113572, 2.791495, -1.495652 };  //1M +x
	FLT qNLH1[4] = { 0.807419, 0.372818, -0.451339, 0.073308 };

	FLT pLH2[3] = {0.153580, 3.251673, -0.190491};
	FLT qLH2[4] = {0.217017, 0.482214, 0.306568, 0.791448 };
	FLT pNLH2[3] = {-0.175330, 3.351943, 0.669623 };
	FLT qNLH2[4] = {0.257241, 0.394159, 0.292555, 0.832392 };
#endif

	FLT pOut1[3];
	FLT pOut2[3];

	qLH1[0] *= -1;
	qLH2[0] *= -1;

	quatrotatevector( pOut1, qLH1, pLH1 );
	quatrotatevector( pOut2, qLH2, pLH2 );

	printf( "%f %f %f\n", PFTHREE( pOut1 ) );
	printf( "%f %f %f\n", PFTHREE( pOut2 ) );

	return 0;

#endif
	
}

