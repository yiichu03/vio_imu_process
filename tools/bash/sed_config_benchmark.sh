#!/usr/bin/env bash
# change configuration file for benchmarking. Notice that 
# the camera and IMU parameters are not touched.

sed -i "/numImuFrames:/c\numImuFrames: $NUM_IMU_FRAMES" $SWIFT_VIO_TEMPLATE
sed -i "/projection_intrinsic_rep:/c\        projection_intrinsic_rep: FIXED," $SWIFT_VIO_TEMPLATE
sed -i "/extrinsic_rep:/c\        extrinsic_rep: FIXED," $SWIFT_VIO_TEMPLATE

sed -i "/algorithm/c\    algorithm: $ESTIMATOR_ALGORITHM" $SWIFT_VIO_TEMPLATE
sed -i "/useEpipolarConstraint/c\    useEpipolarConstraint: $useEpipolarConstraint" $SWIFT_VIO_TEMPLATE
sed -i "/cameraObservationModelId/c\    cameraObservationModelId: $cameraObservationModelId" $SWIFT_VIO_TEMPLATE
sed -i "/landmarkModelId/c\    landmarkModelId: $landmarkModelId" $SWIFT_VIO_TEMPLATE
