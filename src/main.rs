use crate::ik::{FabrikChain, KinematicsMode, MotionHueristics};

use bevy::prelude::*;
use bevy_transform_gizmo::{GizmoUpdate, TransformGizmoEvent};

mod ik;

#[derive(Resource)]
pub struct UiState {
    lock_ground: bool,
    kinematics_mode: KinematicsMode
}

impl Default for UiState {
    fn default() -> Self {
        Self { lock_ground: true, kinematics_mode: KinematicsMode::InverseKinematics }
    }
}

#[derive(Component)]
pub struct LimbData(FabrikChain);

impl LimbData {
    fn get_mut(&mut self, state: &LimbState) -> &mut FabrikChain {
        match state {
            LimbState::RealLimb => &mut self.0,
            LimbState::FantasyLimb => self.0.limb.as_mut().unwrap(),
        }
    }
    
    fn get(&self, state: &LimbState) -> &FabrikChain {
        match state {
            LimbState::RealLimb => &self.0,
            LimbState::FantasyLimb => &self.0.limb.as_ref().unwrap(),
        }
    }
}

#[derive(Component, Default)]
pub struct VelocityDisplay(Vec<Vec<f32>>);

fn main() {
    println!("Hello, world!");
}

// fn forwared_kinematics(kinematics_mode: Res<KinematicsMode>) {
//     if *kinematics_mode != KinematicsMode::ForwardKinematics {
//         return;
//     }
// }

#[derive(Component, Default, Debug, Clone)]
struct ControlBall {
    index: usize,
}

#[derive(Component, Default, Debug, Clone)]
struct InnerBall {
    index: usize,
}

#[derive(Component, Default, Debug, Clone)]
struct Segment {
    index: usize,
}

#[derive(Component, Default, Debug, Clone)]
struct FantasyComponent;

#[derive(Event, Default, Message)]
struct SyncTransform;

#[derive(Event, Default, Message)]
struct RecomputeLimb;

#[derive(Event, Default)]
struct MoveLimb;

#[derive(Event, Message)]
struct GizmoUpdater(GizmoUpdate);

#[derive(States, Default, Debug, Hash, PartialEq, Eq, Clone, strum::EnumIter, strum::Display)]
enum LimbState {
    #[default]
    RealLimb,
    FantasyLimb,
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut ev_sync_transforms: EventWriter<SyncTransform>
) {
    let joints = vec![
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(2.0, 0.0, 0.0),
        Vec3::new(3.0, 0.0, 0.0),
        Vec3::new(4.0, 0.0, 0.0),
    ];
    let mut limb = FabrikChain::new(joints, MotionHueristics::default());
    commands.spawn(VelocityDisplay::default());
    
    commands.spawn((
        PointLight {
            intensity: 9000.0,
            range: 100.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(8.0, 16.0, 8.0),
    ));
    
    let control_ball_mesh = meshes.add(
        Mesh::from(Sphere::new(0.29).mesh().uv(32, 18))
    );
    let ball_mesh = meshes.add(
        Mesh::from(Sphere::new(0.3).mesh().uv(32, 18))
    );
    let fantasy_ball_mesh = meshes.add(
        Mesh::from(Sphere::new(0.3 * 0.999).mesh().uv(32, 18))
    );
    
    let material = materials.add(StandardMaterial::default());
    let fantasy_material = materials.add(StandardMaterial {
        base_color: Color::linear_rgba(0.19, 0.0, 0.5, 1.0),
        ..default()
    });
    let transculent_material = materials.add(StandardMaterial {
        alpha_mode: AlphaMode::Mask(0.5),
        base_color: Color::linear_rgba(0.7, 0.7, 1.0, 0.2),
        ..default()
    });
}

fn sync_ball_transform(
    mut query_chain: Query<&mut LimbData>,
    mut query_ball: Query<(&InnerBall, &mut Transform), Without<FantasyComponent>>,
    mut query_ball_fantasy: Query<(&InnerBall, &mut Transform), With<FantasyComponent>>
) {
    let chain = query_chain.single_mut().unwrap();
    
    for (ball, mut transform) in query_ball.iter_mut() {
        *transform = Transform::from_translation(chain.0.joints[ball.index]);
    }
    
    for (ball, mut transform) in query_ball_fantasy.iter_mut() {
        *transform = Transform::from_translation(chain.0.limb.as_ref().unwrap().joints[ball.index]);
    }
}

fn handle_limb_switch(mut ev_sync_transforms: EventWriter<SyncTransform>) {
    ev_sync_transforms.write_default();
}

fn sync_ctrl_ball_transform(
    mut query_chain: Query<&mut LimbData>,
    mut query_ctrl_ball: Query<(&ControlBall, &mut Transform)>,
    limb_state: Res<State<LimbState>>,
) {
    let chain = query_chain.single_mut().unwrap();
    let limb = chain.get(&limb_state.get());
    
    for (ctrl_ball, mut transform) in query_ctrl_ball.iter_mut() {
        *transform = Transform::from_translation(limb.joints[ctrl_ball.index]);
    }
}

fn sync_segment_transform(
    mut query_chain: Query<&mut LimbData>,
    mut query_segment: Query<(&Segment, &mut Transform), Without<FantasyComponent>>,
    mut query_segment_fantasy: Query<(&Segment, &mut Transform), With<FantasyComponent>>
) {
    let chain = query_chain.single_mut().unwrap();
    for (segment, mut transform) in query_segment.iter_mut() {
        *transform = chain.0.segment_transforms[segment.index];
    }
    for (segment, mut transform) in query_segment_fantasy.iter_mut() {
        *transform = chain.0.limb.as_ref().unwrap().segment_transforms[segment.index];
    }
}

fn move_limb(
    query_ctrl_ball: Query<(&ControlBall, &Transform)>,
    mut query_chain: Query<&mut LimbData>,
    mut ev_gizmo: EventReader<GizmoUpdater>,
    mut ev_recompute: EventWriter<RecomputeLimb>,
    limb_state: Res<State<LimbState>>
) {
    let mut excluded = Vec::new();
    if ev_gizmo.is_empty() { return; }
    
    let mut chain = query_chain.single_mut().unwrap();
    let limb = chain.get_mut(limb_state.get());
    limb.targets.clear();
    
    for &event in ev_gizmo.read() {
        let (ball, transform) = query_ctrl_ball
            .get(event.0.entity())
            .expect("Something is moving but it's not a ball!");
        excluded.push(ball.index);
        limb.targets
            .push((ball.index, transform.translation.clone()));
    }
}