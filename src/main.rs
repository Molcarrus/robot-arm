use crate::ik::{FabrikChain, KinematicsMode, MotionHueristics, PoseDiscrepancy};

use bevy::prelude::*;
use bevy_egui::{EguiContexts, egui::Window};
use bevy_transform_gizmo::{GizmoUpdate, TransformGizmoEvent};
use egui_plot::{Line, Plot, PlotPoints};
use strum::IntoEnumIterator;

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

#[derive(Component, Default, Debug, Clone, Copy)]
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

#[derive(Default, Message)]
struct SyncTransform;

#[derive(Default, Message)]
struct RecomputeLimb;

#[derive(Default, Message)]
struct MoveLimb;

#[derive(Message)]
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
    mut ev_sync_transforms: MessageWriter<SyncTransform>
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

fn handle_limb_switch(mut ev_sync_transforms: MessageWriter<SyncTransform>) {
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
    mut ev_gizmo: MessageReader<GizmoUpdater>,
    mut ev_recompute: MessageWriter<RecomputeLimb>,
    limb_state: Res<State<LimbState>>
) {
    let mut excluded = Vec::new();
    if ev_gizmo.is_empty() { return; }
    
    let mut chain = query_chain.single_mut().unwrap();
    let limb = chain.get_mut(limb_state.get());
    limb.targets.clear();
    
    for &event in ev_gizmo.read() {
        let entity = event.0.entity().clone();
        let (ball, transform) = query_ctrl_ball
            .get(entity)
            .expect("Something is moving but it's not a ball!");
        excluded.push(ball.index);
        limb.targets
            .push((ball.index, transform.translation.clone()));
    }
}

fn recompute_limb(
    mut query_chain: Query<&mut LimbData>,
    mut query_velocity_display: Query<&mut VelocityDisplay>,
    mut ev_sync_transform: MessageWriter<SyncTransform>,
    limb_state: Res<State<LimbState>>
) {
    let mut chain = query_chain.single_mut().unwrap();
    let limb = chain.get_mut(limb_state.get());
    
    limb.solve(10, PoseDiscrepancy::default(), &mut KinematicsMode::InverseKinematics);
    
    if !limb.angular_velocities.is_empty() {
        query_velocity_display
            .single_mut()
            .unwrap()
            .0
            .push(limb.angular_velocities.clone());
    }
    
    ev_sync_transform.write_default();
}

fn diplay_ui(
    mut context: EguiContexts,
    mut query: Query<&mut VelocityDisplay>,
    mut query_chain: Query<&mut LimbData>,
    mut ui_state: ResMut<UiState>,
    mut ev_sync_transforms: MessageWriter<SyncTransform>,
    limb_state_ro: ResMut<State<LimbState>>,
    mut limb_state: ResMut<NextState<LimbState>>
) {
    let mut chain = query_chain.single_mut().unwrap();
    
    Window::new("Limb Control").show(context.ctx_mut().unwrap(), |ui| {
        let mut velocity_display = query.single_mut().unwrap();
        
        if ui
            .button("Reset Graph")
            .clicked() 
        {
            velocity_display.0.clear();
        }
        if ui
            .button("Reset All")
            .clicked() 
        {
            velocity_display.0.clear();
            chain.0.reset();
            ev_sync_transforms.write_default();
        }
        if ui
            .checkbox(&mut ui_state.lock_ground, "Lock Ground")
            .changed()
        {
            chain.0.lock_ground = ui_state.lock_ground;
            chain.0.limb.as_mut().unwrap().lock_ground = ui_state.lock_ground;
        }
        
        for possible_mode in LimbState::iter() {
            let name = possible_mode.to_string();
            if ui
                .radio_value(&mut limb_state_ro.clone(), possible_mode.clone(), name)
                .clicked()
            {
                limb_state.set(possible_mode);
            }
        }
        
        ui.separator();
        
        let mut velocities = Vec::new();
        if let Some(first_len) = velocity_display.0.first().map(|x| x.len()) {
            for _ in 0..first_len {
                velocities.push(Vec::new());
            }
            for x in 0..velocity_display.0.len() {
                for y in 0..velocity_display.0[x].len() {
                    let new_point = [x as f64, velocity_display.0[x][y] as f64];
                    match velocities.get_mut(y) {
                        Some(y_ptr) => {
                            y_ptr.push(new_point);
                        }
                        None => {
                            velocities.push(vec![new_point]);
                        }
                    }
                }
            }
            
            let lines = velocities
                .into_iter()
                .map(|x| Line::new("Plot #1", PlotPoints::new(x)));
            
            Plot::new("velocity")
                .view_aspect(2.0)
                .show(ui, |plot_ui| {
                    for line in lines {
                        plot_ui.line(line);
                    }
                });
        } else {
            ui.label("NO DATA");
        }
    });
}