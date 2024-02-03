use std::f32::consts::PI;

use rapier3d::{na::{UnitQuaternion, UnitVector3}, prelude::*};
use rapier_testbed3d::Testbed;

pub fn init_world(testbed: &mut Testbed) {
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();

    let ground = RigidBodyBuilder::fixed().translation(vector![0.0, 0.0, 0.0]);
    let body = bodies.insert(ground);
    let collider = ColliderBuilder::cuboid(1.0, 1.0, 1.0);
    colliders.insert_with_parent(collider, body, &mut bodies);

    let capsule_pos = vector![0., 4., 0.];
    let rb1 = RigidBodyBuilder::dynamic().position(capsule_pos.into());
    let rb1_handle = bodies.insert(rb1);
    let collider = ColliderBuilder::capsule_y(0.5, 0.1);
    colliders.insert_with_parent(collider, rb1_handle, &mut bodies);

    let cube_pos = vector![1., 5., 2.];
    let rb2 = RigidBodyBuilder::fixed().position(cube_pos.into());
    let rb2_handle = bodies.insert(rb2);
    let collider = ColliderBuilder::cuboid(0.5, 0.5, 0.5);
    colliders.insert_with_parent(collider, rb2_handle, &mut bodies);

    //todo:
    //make obj at fixed position
    //calculate quaternion to point capsule at it
    //convert quat to euler angles
    //set those angles as joint motor positions
    //(might have to swap around the coordinates a bit)

    let target_dir = cube_pos - capsule_pos;
    let initial_dir = vector![0., 1., 0.];
    let target_rot = UnitQuaternion::from_axis_angle(
        &UnitVector3::new_normalize(initial_dir.cross(&target_dir)),
        initial_dir.angle(&target_dir)
    );
    let (x, y, z) = target_rot.to_rotation_matrix().euler_angles();
    
    let joint = SphericalJointBuilder::new()
        .local_anchor1(point![0.0, 4.0, 0.0])
        .local_anchor2(point![0.0, 0.0, 0.0])
        .motor_position(JointAxis::AngX, x + 200.*PI, 1000.0, 200.)
        .motor_position(JointAxis::AngY, y, 1000.0, 200.)
        .motor_position(JointAxis::AngZ, z, 1000.0, 200.)
        ;

    multibody_joints.insert(body, rb1_handle, joint, true);

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![20.0, 0.0, 0.0], point![0.0, 0.0, 0.0]);
}
