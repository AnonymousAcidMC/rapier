use std::f32::consts::{FRAC_PI_4, PI};

use rapier3d::prelude::*;
use rapier_testbed3d::Testbed;

pub fn init_world(testbed: &mut Testbed) {
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();

    let ground_size = 5.;
    let ground_height = 0.1;
    let ground = RigidBodyBuilder::fixed().translation(vector![0.0, -ground_height, 0.0]);
    let body = bodies.insert(ground);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size);
    colliders.insert_with_parent(collider, body, &mut bodies);

    let segment_len = 0.5;
    let radius = 0.1;
    let half_height = (segment_len-radius*2.)/2.;
    let segment_colldier = ColliderBuilder::capsule_y(half_height, radius);
    
    let spine_root = bodies.insert(RigidBodyBuilder::fixed().translation(vector![0., 3., 0.]));
    {//branch 1
        let seg1 = bodies.insert(RigidBodyBuilder::dynamic());
        let seg2 = bodies.insert(RigidBodyBuilder::dynamic());

        colliders.insert_with_parent(segment_colldier.clone(), spine_root, &mut bodies);
        colliders.insert_with_parent(segment_colldier.clone(), seg1, &mut bodies);
        colliders.insert_with_parent(segment_colldier.clone(), seg2, &mut bodies);

        let joint = SphericalJointBuilder::new()
            .local_anchor1(point![0., segment_len/2., 0.])
            .local_anchor2(point![0., -segment_len/2., 0.]);

        multibody_joints.insert(spine_root, seg1, joint, true);
        multibody_joints.insert(seg1, seg2, joint, true);
    }

    let mbj_handle;
    {//branch 2 (moving)
        let seg3 = bodies.insert(RigidBodyBuilder::dynamic());
        colliders.insert_with_parent(
            ColliderBuilder::capsule_x(half_height, radius),
            seg3, 
            &mut bodies
        );
        
        let joint = RevoluteJointBuilder::new(Vector::x_axis())
            .local_anchor1(point![0., segment_len/2., 0.])
            .local_anchor2(point![-segment_len/2., 0., 0.])
            .motor(PI, FRAC_PI_4, 1., 0.05);

        mbj_handle = multibody_joints.insert(spine_root, seg3, joint, true).unwrap();
    }

    let mb = multibody_joints.get_mut(mbj_handle).unwrap().0;
        mb.set_self_contacts_enabled(false);

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![20.0, 20.0, 20.0], point![0.0, 0.0, 0.0]);
}
