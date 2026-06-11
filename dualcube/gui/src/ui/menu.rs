//! The top panel: the main menu bar and the pipeline stage bar.

use super::theme::{colored_text, to_color32, BLUE, RED, TEXT_COLOR, TEXT_COLOR2};
use super::widgets::{
    label, log_slider, menu_button, radio, sep, sleek_button, sleek_button_unfocused,
    sleek_button_warn, slider, space,
};
use crate::colors;
use crate::controls::InteractiveMode;
use crate::jobs::{Job, JobRequest, JobState};
use crate::render::store::RenderObjectSettingStore;
use crate::render::Objects;
use crate::resources::{Configuration, InputResource, Phase, SolutionResource};
use bevy::prelude::*;
use bevy_egui::egui::{Align, CursorIcon, Frame, Layout, MenuBar, TopBottomPanel, Ui};
use bevy_orbit_camera::automatic::AutomaticRotation;
use dualcube::prelude::*;
use std::path::PathBuf;

const TEXT_SIZE: f32 = 12.;

/// Visibility presets: which features of each object should be visible.
const PRESETS: [(&str, fn(Objects) -> &'static [&'static str]); 4] = [
    ("> Grayscale", |object| match object {
        Objects::InputMesh | Objects::QuadMesh => &["gray", "wireframe"],
        Objects::Polycube => &["gray", "paths", "flat paths"],
        Objects::PolycubeMap => &["colored", "triangles"],
    }),
    ("> Dual", |object| match object {
        Objects::InputMesh | Objects::Polycube => &["black", "x-loops", "y-loops", "z-loops"],
        Objects::PolycubeMap => &["colored", "triangles"],
        Objects::QuadMesh => &["gray", "paths", "flat paths"],
    }),
    ("> Primal", |object| match object {
        Objects::InputMesh => &["segmentation", "paths", "flat paths", "wireframe"],
        Objects::Polycube => &["colored", "paths", "flat paths"],
        Objects::PolycubeMap => &["colored", "triangles"],
        Objects::QuadMesh => &["colored", "wireframe", "paths", "flat paths"],
    }),
    ("> Vector fields", |object| match object {
        Objects::InputMesh => &["black", "x-field", "y-field", "z-field"],
        Objects::Polycube => &[],
        Objects::PolycubeMap => &[],
        Objects::QuadMesh => &[],
    }),
];

fn apply_render_preset(
    store: &mut RenderObjectSettingStore,
    preset: fn(Objects) -> &'static [&'static str],
) {
    for (&object, settings) in store.objects.iter_mut() {
        let show = preset(object);
        for (label, setting) in settings.settings.iter_mut() {
            setting.visible = show.contains(&label.as_str());
        }
    }
}

/// One pipeline stage: a menu with its actions (if available), otherwise a
/// grayed-out label, followed by an optional status.
fn stage(
    ui: &mut Ui,
    stopped: bool,
    name: &str,
    available: bool,
    status: Option<String>,
    menu: impl FnOnce(&mut Ui),
) {
    if available && !stopped {
        menu_button(ui, name, menu);
        if let Some(status) = status {
            label(ui, &status, TEXT_SIZE, TEXT_COLOR2);
        }
    } else {
        label(ui, name, TEXT_SIZE, TEXT_COLOR2);
    }
}

/// Toggle between continuing past a phase or stopping the pipeline there.
fn stop_toggle(ui: &mut Ui, conf: &mut Configuration, stopped: &mut bool, phase: Phase) {
    if conf.stop == phase {
        *stopped = true;
        if sleek_button_warn(ui, "  🚫  ") {
            conf.stop = Phase::None;
        }
    } else if sleek_button_unfocused(ui, "─────") {
        conf.stop = phase;
    }
}

/// Shows the top panel: the menu bar and the pipeline stage bar.
pub fn show(
    mut egui_ctx: bevy_egui::EguiContexts,
    mut jobs: MessageWriter<JobRequest>,
    mut conf: ResMut<Configuration>,
    job_state: Res<JobState>,
    solution: Res<SolutionResource>,
    mut render_setting_store: ResMut<RenderObjectSettingStore>,
    mesh_ref: Res<InputResource>,
    mut automatic_rotation: ResMut<AutomaticRotation>,
) -> Result<(), BevyError> {
    TopBottomPanel::top("panel")
        .show_separator_line(false)
        .show(egui_ctx.ctx_mut()?, |ui| {
            ui.add_space(10.);

            if job_state.request.is_some() {
                ui.output_mut(|o| o.cursor_icon = CursorIcon::Progress);
            }

            ui.horizontal(|ui| {
                ui.with_layout(Layout::top_down(Align::Center), |ui| {
                    ui.with_layout(Layout::left_to_right(Align::TOP), |ui| {
                        Frame {
                            outer_margin: bevy_egui::egui::epaint::Margin::symmetric(15, 0),
                            shadow: bevy_egui::egui::epaint::Shadow::NONE,
                            ..default()
                        }
                        .show(ui, |ui| {
                            MenuBar::new().ui(ui, |ui| {
                                menu_bar(
                                    ui,
                                    &mut jobs,
                                    &mut conf,
                                    &solution,
                                    &mut render_setting_store,
                                    &mut automatic_rotation,
                                );
                            });
                        });
                    });

                    ui.add_space(5.);
                    ui.separator();
                    ui.add_space(5.);

                    ui.with_layout(Layout::left_to_right(Align::TOP), |ui| {
                        ui.add_space(17.);
                        MenuBar::new().ui(ui, |ui| {
                            pipeline_bar(ui, &mut jobs, &mut conf, &solution, &mesh_ref);
                        });
                    });

                    sep(ui);
                });
            });
        });

    Ok(())
}

/// The File / Rendering / Camera / Manual menus.
fn menu_bar(
    ui: &mut Ui,
    jobs: &mut MessageWriter<JobRequest>,
    conf: &mut Configuration,
    solution: &SolutionResource,
    render_setting_store: &mut RenderObjectSettingStore,
    automatic_rotation: &mut AutomaticRotation,
) {
    menu_button(ui, "File", |ui| {
        space(ui);
        if sleek_button(ui, "Load") {
            if let Some(path) = rfd::FileDialog::new()
                .add_filter(
                    "obj, SAVE FILE for loops, or SAVE FILE for dualcube",
                    &["obj", "loops", "dc"],
                )
                .pick_file()
            {
                jobs.write(JobRequest::Run(Job::import(path)));
            }
        }
        sep(ui);
        if sleek_button(ui, "Export") {
            if let Some(path) = rfd::FileDialog::new().save_file() {
                jobs.write(JobRequest::Run(Job::export(
                    solution.current_solution.clone(),
                    path,
                )));
            }
        }
        space(ui);
        if sleek_button(ui, "Export (NLR)") {
            if let Some(path) = rfd::FileDialog::new().save_file() {
                jobs.write(JobRequest::Run(Job::export_nlr(
                    solution.current_solution.clone(),
                    path,
                )));
            }
        }
        space(ui);
        if sleek_button(ui, "Export (Honors)") {
            if let Some(path) = rfd::FileDialog::new().save_file() {
                jobs.write(JobRequest::Run(Job::export_dotgraph(
                    solution.current_solution.clone(),
                    path,
                )));
            }
        }
        sep(ui);
        if sleek_button(ui, "Quit") {
            std::process::exit(0);
        }
        space(ui);
    });

    space(ui);

    menu_button(ui, "Rendering", |ui| {
        space(ui);
        label(ui, "Presets", TEXT_SIZE, TEXT_COLOR);
        for (name, preset) in PRESETS {
            space(ui);
            if sleek_button(ui, name) {
                apply_render_preset(render_setting_store, preset);
            }
        }
    });

    space(ui);

    menu_button(ui, "Camera", |ui| {
        space(ui);
        label(ui, "Automatic camera rotation", TEXT_SIZE, TEXT_COLOR);
        space(ui);

        ui.checkbox(&mut automatic_rotation.enabled, "enabled");
        slider(
            ui,
            "sensitivity",
            &mut automatic_rotation.sensitivity,
            -std::f32::consts::PI..=std::f32::consts::PI,
            false,
        );

        sep(ui);
        label(
            ui,
            "Manual camera control sensitivity",
            TEXT_SIZE,
            TEXT_COLOR,
        );
        space(ui);

        log_slider(ui, "rotate", &mut conf.camera_rotate_sensitivity, 1.);
        space(ui);
        log_slider(ui, "translate", &mut conf.camera_translate_sensitivity, 3.);
        space(ui);
        log_slider(ui, "zoom", &mut conf.camera_zoom_sensitivity, 1.);
        space(ui);

        if sleek_button(ui, "High-precision mode") {
            conf.camera_rotate_sensitivity = 0.01;
            conf.camera_translate_sensitivity = 0.01;
            conf.camera_zoom_sensitivity = 0.01;
        }

        space(ui);

        if sleek_button(ui, "Reset to default") {
            conf.camera_rotate_sensitivity = 0.2;
            conf.camera_translate_sensitivity = 2.0;
            conf.camera_zoom_sensitivity = 0.2;
        }

        space(ui);

        if sleek_button(ui, "X up") {
            conf.camera_up = Vec3::X;
        }
        if sleek_button(ui, "Y up") {
            conf.camera_up = Vec3::Y;
        }
        if sleek_button(ui, "Z up") {
            conf.camera_up = Vec3::Z;
        }

        space(ui);
    });

    space(ui);

    menu_button(ui, "Manual", |ui| {
        space(ui);

        let params = &mut conf.fields_params;
        slider(
            ui,
            "outer_iterations",
            &mut params.outer_iterations,
            0..=50,
            false,
        );
        slider(
            ui,
            "cg_iterations",
            &mut params.cg_iterations,
            10..=500,
            false,
        );
        slider(
            ui,
            "cg_tolerance",
            &mut params.cg_tolerance,
            1e-10..=1e-3,
            true,
        );
        slider(
            ui,
            "smooth_weight",
            &mut params.smooth_weight,
            1e-3..=100.0,
            true,
        );
        slider(
            ui,
            "axis_weight",
            &mut params.axis_weight,
            0.0..=100.0,
            true,
        );
        slider(
            ui,
            "curvature_weight",
            &mut params.curvature_weight,
            0.0..=100.0,
            true,
        );
        slider(
            ui,
            "coupling_weight",
            &mut params.coupling_weight,
            0.0..=0.5,
            true,
        );
        slider(
            ui,
            "damping_weight",
            &mut params.damping_weight,
            1e-6..=1.0,
            true,
        );

        if sleek_button(ui, "FIELDS") {
            jobs.write(JobRequest::Run(Job::fields(
                solution.current_solution.clone(),
                conf.clone(),
            )));
        }

        space(ui);

        if conf.interactive_mode == InteractiveMode::LoopModification {
            if sleek_button(ui, "Modify loops [active]") {
                conf.interactive_mode = InteractiveMode::None;
            }
        } else if sleek_button_unfocused(ui, "Modify loops [not active]") {
            conf.interactive_mode = InteractiveMode::LoopModification;
        }

        space(ui);

        for direction in [
            PrincipalDirection::X,
            PrincipalDirection::Y,
            PrincipalDirection::Z,
        ] {
            let color = to_color32(colors::from_direction(
                direction,
                Some(Perspective::Dual),
                None,
            ));
            radio(ui, &mut conf.direction, direction, color);
            space(ui);
        }

        slider(ui, "alpha", &mut conf.alpha, 0.0..=1.0, false);

        space(ui);

        if let Some(edgepair) = conf.selected {
            if let Some(Some(sol)) = solution.next[conf.direction as usize].get(&edgepair) {
                ui.label("DUAL[");
                if sol.dual.is_ok() {
                    ui.label(colored_text("Ok", BLUE));
                } else {
                    ui.label(colored_text(&format!("{:?}", sol.dual.as_ref().err()), RED));
                }
                ui.label("]");

                ui.label("EMBD[");
                if sol.layout.is_some() {
                    ui.label(colored_text("Ok", BLUE));
                } else {
                    ui.label(colored_text("Not found", RED));
                }
                ui.label("]");
            }
        }

        sep(ui);

        if conf.interactive_mode == InteractiveMode::SegmentationModification {
            if sleek_button(ui, "Modify segmentation [active]") {
                conf.interactive_mode = InteractiveMode::None;
            }
        } else if sleek_button_unfocused(ui, "Modify segmentation [not active]") {
            conf.interactive_mode = InteractiveMode::SegmentationModification;
        }

        space(ui);
    });

    if sleek_button(ui, "Hex") {
        jobs.write(JobRequest::Run(Job::export_hex(
            solution.current_solution.clone(),
            PathBuf::from("./out/temp2319701278924168937120"),
        )));
    }
}

/// The pipeline bar: one stage per phase, each with its actions and a stop
/// toggle in between.
fn pipeline_bar(
    ui: &mut Ui,
    jobs: &mut MessageWriter<JobRequest>,
    conf: &mut Configuration,
    solution: &SolutionResource,
    mesh_ref: &InputResource,
) {
    let current = &solution.current_solution;
    let mut stopped = false;

    // Input
    label(ui, "Input", TEXT_SIZE, TEXT_COLOR);
    label(
        ui,
        &format!("({})", current.mesh_ref.nr_verts()),
        TEXT_SIZE,
        TEXT_COLOR2,
    );
    stop_toggle(ui, conf, &mut stopped, Phase::Input);

    // Loops
    stage(
        ui,
        stopped,
        "Loops",
        current.mesh_ref.nr_verts() != 0,
        Some(format!("({})", current.loops.len())),
        |ui| {
            if sleek_button(ui, "initialize") {
                jobs.write(JobRequest::Run(Job::initialize_loops(
                    current.clone(),
                    mesh_ref.flow_graphs.clone(),
                    conf.clone(),
                )));
                ui.close();
            }
            if sleek_button(ui, "evolve") {
                jobs.write(JobRequest::Run(Job::evolve(
                    current.clone(),
                    mesh_ref.flow_graphs.clone(),
                    conf.clone(),
                )));
                ui.close();
            }
            slider(ui, "iterations", &mut conf.iterations, 1..=20, false);
            slider(ui, "pool1", &mut conf.pool1, 1..=20, false);
            slider(ui, "pool2", &mut conf.pool2, 1..=50, false);
        },
    );
    stop_toggle(ui, conf, &mut stopped, Phase::Loops);

    // Dual
    stage(
        ui,
        stopped,
        "Dual",
        !current.loops.is_empty(),
        Some(
            if current.dual.is_ok() {
                "(Ok)"
            } else {
                "(err)"
            }
            .to_string(),
        ),
        |ui| {
            if sleek_button(ui, "(re)compute") {
                jobs.write(JobRequest::Run(Job::compute_dual(
                    current.clone(),
                    conf.clone(),
                )));
                ui.close();
            }
        },
    );
    stop_toggle(ui, conf, &mut stopped, Phase::Dual);

    // Layout
    let layout_status = match &current.layout {
        Some(layout) => match (layout.alignment, layout.orthogonality) {
            (Some(alignment), Some(orthogonality)) => {
                format!("({alignment:.3}, {orthogonality:.3})")
            }
            _ => "(Quality missing(?))".to_string(),
        },
        None => "(None)".to_string(),
    };
    stage(
        ui,
        stopped,
        "Layout",
        current.dual.is_ok(),
        Some(layout_status),
        |ui| {
            if sleek_button(ui, "(re)compute corners") {
                jobs.write(JobRequest::Run(Job::place_corners(
                    current.clone(),
                    conf.clone(),
                )));
                ui.close();
            }
            if sleek_button(ui, "optimize corners") {
                jobs.write(JobRequest::Run(Job::smoothen_layout(
                    current.clone(),
                    conf.clone(),
                )));
                ui.close();
            }
            space(ui);
            if sleek_button(ui, "(re)compute paths") {
                jobs.write(JobRequest::Run(Job::place_paths(
                    current.clone(),
                    conf.clone(),
                )));
                ui.close();
            }
            if sleek_button(ui, "optimize paths") {
                jobs.write(JobRequest::Run(Job::path_straightening(
                    current.clone(),
                    conf.clone(),
                )));
                ui.close();
            }
        },
    );
    stop_toggle(ui, conf, &mut stopped, Phase::Layout);

    // Polycube
    stage(
        ui,
        stopped,
        "Polycube",
        current.layout.is_some(),
        None,
        |ui| {
            ui.checkbox(&mut conf.unit, "unit");
            if sleek_button(ui, "(re)compute") {
                jobs.write(JobRequest::Run(Job::compute_polycube(
                    current.clone(),
                    conf.clone(),
                )));
                ui.close();
            }
        },
    );
    stop_toggle(ui, conf, &mut stopped, Phase::Polycube);

    // Quad
    stage(
        ui,
        stopped,
        "Quad",
        true,
        Some(if current.quad.is_some() { "(Ok)" } else { "" }.to_string()),
        |ui| {
            if sleek_button(ui, "(re)compute") {
                jobs.write(JobRequest::Run(Job::compute_quad(
                    current.clone(),
                    conf.clone(),
                )));
                ui.close();
            }
            slider(ui, "omega", &mut conf.omega, 1..=20, false);
        },
    );
}
