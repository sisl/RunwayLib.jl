#ifndef LIBPOSEEST_H_
#define LIBPOSEEST_H_

#ifdef __cplusplus
extern "C" {
#endif

// Data structures for C API
typedef struct {
    double x, y, z;
} WorldPoint_C;

typedef struct {
    double x, y;
} ProjectionPoint_C;

typedef struct {
    double yaw, pitch, roll;
} Rotation_C;

typedef struct {
    WorldPoint_C position;
    Rotation_C rotation;
    double residual_norm;
    int converged;
} PoseEstimate_C;

typedef enum {
    CAMERA_CONFIG_CENTERED = 0,
    CAMERA_CONFIG_OFFSET = 1
} CameraConfigType;

// Error codes
#define POSEEST_SUCCESS 0
#define POSEEST_ERROR_INVALID_INPUT -1
#define POSEEST_ERROR_BEHIND_CAMERA -2
#define POSEEST_ERROR_NO_CONVERGENCE -3
#define POSEEST_ERROR_INSUFFICIENT_POINTS -4

// Function declarations

// Simple test function (existing)
int test_estimators();

// Enhanced API functions
int estimate_pose_6dof(
    const WorldPoint_C* runway_corners,
    const ProjectionPoint_C* projections,
    int num_points,
    CameraConfigType camera_config,
    PoseEstimate_C* result
);

int estimate_pose_3dof(
    const WorldPoint_C* runway_corners,
    const ProjectionPoint_C* projections,
    int num_points,
    const Rotation_C* known_rotation,
    CameraConfigType camera_config,
    PoseEstimate_C* result
);

// Utility functions
int project_point(
    const WorldPoint_C* camera_position,
    const Rotation_C* camera_rotation,
    const WorldPoint_C* world_point,
    CameraConfigType camera_config,
    ProjectionPoint_C* result
);

// Library initialization (sets JULIA_DEPOT_PATH)
int initialize_poseest_library(const char* depot_path);

// Get error message for error code
const char* get_error_message(int error_code);

#ifdef __cplusplus
}
#endif

#endif // LIBPOSEEST_H_
