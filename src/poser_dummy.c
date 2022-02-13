#include <survive.h>
#include <stdio.h>
#include <stdlib.h>

typedef struct
{
	int something;
	//Stuff
} DummyData;

int PoserDummy(SurviveObject *so, PoserData *pd) {
	PoserType pt = pd->pt;
	SurviveContext * ctx = so->ctx;
	void **user = survive_object_plugin_data(so, PoserDummy);
	DummyData *dd = *user;

	if (!dd)
		*user = dd = SV_MALLOC(sizeof(DummyData));

	switch( pt )
	{
	case POSERDATA_IMU:
	{
		PoserDataIMU * imu = (PoserDataIMU*)pd;
		SurvivePose pose = LinmathPose_Identity;
		PoserData_poser_pose_func(pd, so, &pose, -1, 0);
		//printf( "IMU:%s (%f %f %f) (%f %f %f)\n", so->codename, imu->accel[0], imu->accel[1], imu->accel[2], imu->gyro[0], imu->gyro[1], imu->gyro[2] );
		break;
	}
	case POSERDATA_LIGHT:
	{
		PoserDataLight * l = (PoserDataLight*)pd;
		//printf( "LIG:%s %d @ %f rad, %f s (AC %d) (TC %d)\n", so->codename, l->sensor_id, l->angle, l->length, l->acode, l->timecode );
		break;
	}
	case POSERDATA_DISASSOCIATE:
	{
		*user = 0;
		free( dd );
		//printf( "Need to disassociate.\n" );
		break;
	}
	}
	return 0;
}

REGISTER_POSER(PoserDummy)
