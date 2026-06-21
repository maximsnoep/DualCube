//! The top panel: the main menu bar and the pipeline stage bar.

use super::theme::*;
use super::widgets::{
    label, log_slider, menu_button, radio, sep, sleek_button, sleek_button_unfocused,
    sleek_button_warn, slider, space,
};
use crate::colors;
use crate::controls::InteractiveMode;
use crate::jobs::{Job, JobState};
use crate::render::Objects;
use crate::render::store::{RenderObjectSetting, RenderObjectSettingStore};
use crate::resources::{Configuration, Phase, SolutionResource};
use bevy::prelude::*;
use bevy_egui::egui::{
    Align, CursorIcon, Frame, Grid, Layout, MenuBar, RichText, TopBottomPanel, Ui,
};
use bevy_orbit_camera::automatic::AutomaticRotation;
use dualcube::prelude::*;
use std::collections::HashMap;

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

/// Hover-preview presets for the pipeline stage buttons.
const PIPELINE_STAGE_PRESETS: [(&str, fn(Objects) -> &'static [&'static str]); 9] = [
    ("Input", |object| match object {
        Objects::InputMesh => &["gray", "wireframe"],
        Objects::Polycube => &["gray", "paths", "flat paths"],
        Objects::PolycubeMap => &["colored", "triangles"],
        Objects::QuadMesh => &["gray", "wireframe"],
    }),
    ("Field", |object| match object {
        Objects::InputMesh => &["black", "wireframe", "x-field", "y-field", "z-field"],
        Objects::Polycube => &["black", "paths", "flat paths"],
        Objects::PolycubeMap => &["colored", "triangles"],
        Objects::QuadMesh => &["gray", "wireframe"],
    }),
    ("Graph", |object| match object {
        Objects::InputMesh => &["black", "wireframe", "x-graph", "y-graph", "z-graph"],
        Objects::Polycube => &["black", "paths", "flat paths"],
        Objects::PolycubeMap => &["colored", "triangles"],
        Objects::QuadMesh => &["gray", "wireframe"],
    }),
    ("Loops", |object| match object {
        Objects::InputMesh | Objects::Polycube => &[
            "black",
            "wireframe",
            "paths",
            "flat paths",
            "x-loops",
            "y-loops",
            "z-loops",
        ],
        Objects::PolycubeMap => &["colored", "triangles"],
        Objects::QuadMesh => &["gray", "paths", "flat paths"],
    }),
    ("Dual", |object| match object {
        Objects::InputMesh | Objects::Polycube => &[
            "black",
            "wireframe",
            "paths",
            "flat paths",
            "x-loops",
            "y-loops",
            "z-loops",
        ],
        Objects::PolycubeMap => &["colored", "triangles"],
        Objects::QuadMesh => &["gray", "paths", "flat paths"],
    }),
    ("Layout", |object| match object {
        Objects::InputMesh => &["segmentation", "paths", "flat paths", "wireframe"],
        Objects::Polycube => &["colored", "paths", "flat paths"],
        Objects::PolycubeMap => &["colored", "triangles"],
        Objects::QuadMesh => &["colored", "wireframe", "paths", "flat paths"],
    }),
    ("Polycube", |object| match object {
        Objects::InputMesh => &["segmentation", "paths", "flat paths", "wireframe"],
        Objects::Polycube => &["colored", "paths", "flat paths"],
        Objects::PolycubeMap => &["colored", "triangles"],
        Objects::QuadMesh => &["colored", "wireframe", "paths", "flat paths"],
    }),
    ("Quad", |object| match object {
        Objects::InputMesh => &["gray", "wireframe"],
        Objects::Polycube => &["gray", "paths", "flat paths"],
        Objects::PolycubeMap => &["colored", "triangles"],
        Objects::QuadMesh => &["gray", "wireframe", "paths", "flat paths"],
    }),
    ("Hex", |object| match object {
        Objects::InputMesh => &["segmentation", "paths", "flat paths", "wireframe"],
        Objects::Polycube => &["colored", "paths", "flat paths"],
        Objects::PolycubeMap => &["colored", "triangles"],
        Objects::QuadMesh => &["colored", "wireframe", "paths", "flat paths"],
    }),
];

/// Look up a hover-preview preset by name.
fn find_preset(name: &str) -> Option<fn(Objects) -> &'static [&'static str]> {
    PIPELINE_STAGE_PRESETS
        .iter()
        .find(|(n, _)| *n == name)
        .map(|(_, preset)| *preset)
}

/// Tracks hover-preview state so that hovering a menu or pipeline button
/// shows a render preview, and unhovering restores the previous settings.
#[derive(Resource, Default)]
pub(crate) struct HoverPreviewState {
    /// Snapshot of render settings taken before the current hover preview.
    pub saved: HashMap<Objects, RenderObjectSetting>,
    /// Name of the currently hovered button (from `PIPELINE_STAGE_PRESETS`).
    pub hovered: Option<&'static str>,
    /// Name of the last clicked (committed) button.
    pub active: Option<&'static str>,
}

fn save_settings(store: &RenderObjectSettingStore) -> HashMap<Objects, RenderObjectSetting> {
    store.objects.clone()
}

fn restore_settings(
    store: &mut RenderObjectSettingStore,
    saved: HashMap<Objects, RenderObjectSetting>,
) {
    store.objects = saved;
}

enum Status {
    Ok(String),
    Err(String),
}

/// One pipeline stage: a menu with its actions (if available), otherwise a
/// grayed-out label, followed by an optional status.
///
/// Returns the menu button's response (so the caller can detect hover/click)
/// when the stage is available and not stopped.
fn stage(
    ui: &mut Ui,
    stopped: bool,
    name: &str,
    available: bool,
    status: Status,
    menu: impl FnOnce(&mut Ui),
) -> Option<bevy_egui::egui::InnerResponse<Option<()>>> {
    if available && !stopped {
        let response = ui.menu_button(RichText::new(name).color(TEXT_COLOR).size(12.), menu);

        match status {
            Status::Ok(status) => label(ui, &status, SMALL_TEXT_SIZE, to_color32(colors::OK_GREEN)),
            Status::Err(err) => label(ui, &err, SMALL_TEXT_SIZE, to_color32(colors::WARN_RED)),
        }
        Some(response)
    } else {
        label(ui, name, REGULAR_TEXT_SIZE, TEXT_COLOR2);
        None
    }
}

/// Toggle between continuing past a phase or stopping the pipeline there.
fn stop_toggle(ui: &mut Ui, conf: &mut Configuration, stopped: &mut bool, phase: Phase) {
    if conf.stop == phase {
        *stopped = true;
        if sleek_button_warn(ui, "   🚫   ") {
            conf.stop = Phase::None;
        }
    } else if sleek_button_unfocused(ui, "────") {
        conf.stop = phase;
    }
}

/// Shows the top panel: the menu bar and the pipeline stage bar.
pub fn show(
    mut egui_ctx: bevy_egui::EguiContexts<'_, '_>,
    mut jobs: MessageWriter<'_, Job>,
    mut conf: ResMut<'_, Configuration>,
    job_state: Res<'_, JobState>,
    solution: Res<'_, SolutionResource>,
    mut render_setting_store: ResMut<'_, RenderObjectSettingStore>,
    mut automatic_rotation: ResMut<'_, AutomaticRotation>,
    mut hover_preview: ResMut<'_, HoverPreviewState>,
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
                            pipeline_bar(
                                ui,
                                &mut jobs,
                                &mut conf,
                                &solution,
                                &mut *render_setting_store,
                                &mut *hover_preview,
                            );
                        });
                    });

                    sep(ui);
                });
            });
        });

    Ok(())
}

/// The File / Camera / Manual menus.
fn menu_bar(
    ui: &mut Ui,
    jobs: &mut MessageWriter<'_, Job>,
    conf: &mut Configuration,
    solution: &SolutionResource,
    automatic_rotation: &mut AutomaticRotation,
) {
    menu_button(ui, "File", |ui| {
        space(ui);
        if sleek_button(ui, "Load") {
            if let Some(path) = rfd::FileDialog::new()
                .add_filter("obj OR dc (dualcube save file)", &["obj", "dc"])
                .pick_file()
            {
                jobs.write(Job::import(path));
            }
        }
        sep(ui);
        if sleek_button(ui, "Export") {
            if let Some(path) = rfd::FileDialog::new().save_file() {
                jobs.write(Job::export(
                    solution.current_solution.clone(),
                    path.with_extension("dc"),
                ));
            }
        }
        #[cfg(feature = "nlr")]
        if sleek_button(ui, "Export (NLR)") {
            if let Some(path) = rfd::FileDialog::new().save_file() {
                jobs.write(Job::export(
                    solution.current_solution.clone(),
                    path.with_extension("nlr"),
                ));
            }
        }
        // space(ui);
        // if sleek_button(ui, "Export (Honors)") {
        //     if let Some(path) = rfd::FileDialog::new().save_file() {
        //         jobs.write(Job::export(
        //             solution.current_solution.clone(),
        //             path.with_extension("apg"),
        //         ));
        //     }
        // }
        sep(ui);
        if sleek_button(ui, "Quit") {
            std::process::exit(0);
        }
        space(ui);
    });

    space(ui);

    menu_button(ui, "Camera", |ui| {
        space(ui);
        label(
            ui,
            "Automatic camera rotation",
            REGULAR_TEXT_SIZE,
            TEXT_COLOR,
        );
        space(ui);

        ui.checkbox(&mut automatic_rotation.enabled, "enabled");
        slider(
            ui,
            "sensitivity",
            &mut automatic_rotation.sensitivity,
            -std::f32::consts::PI..=std::f32::consts::PI,
        );

        sep(ui);
        label(
            ui,
            "Manual camera control sensitivity",
            REGULAR_TEXT_SIZE,
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
        label(ui, "Up axis", REGULAR_TEXT_SIZE, TEXT_COLOR);
        space(ui);

        Grid::new("up_axis_grid")
            .min_col_width(64.)
            .max_col_width(64.)
            .spacing([8.0, 4.0])
            .show(ui, |ui| {
                for &(axis_vec, label, dir) in &[
                    (Vec3::X, "+X", Direction::X),
                    (Vec3::NEG_X, "-X", Direction::X),
                    (Vec3::Y, "+Y", Direction::Y),
                    (Vec3::NEG_Y, "-Y", Direction::Y),
                    (Vec3::Z, "+Z", Direction::Z),
                    (Vec3::NEG_Z, "-Z", Direction::Z),
                ] {
                    let color = to_color32(colors::from_direction(dir, None, None));
                    let is_active = conf.camera_up == axis_vec;
                    let btn_color = if is_active { color } else { TEXT_COLOR2 };
                    if ui
                        .button(RichText::new(label).color(btn_color).size(12.))
                        .clicked()
                    {
                        conf.camera_up = axis_vec;
                    }
                    // After every 2 items, advance to next row
                    if label == "-X" || label == "-Y" || label == "-Z" {
                        ui.end_row();
                    }
                }
            });

        space(ui);
    });

    space(ui);

    menu_button(ui, "Manual", |ui| {
        space(ui);

        if conf.interactive_mode == InteractiveMode::LoopModification {
            if sleek_button(ui, "Modify loops [active]") {
                conf.interactive_mode = InteractiveMode::None;
            }
        } else if sleek_button_unfocused(ui, "Modify loops [not active]") {
            conf.interactive_mode = InteractiveMode::LoopModification;
        }

        space(ui);

        for direction in DIRECTIONS {
            let color = to_color32(colors::from_direction(
                direction,
                Some(Perspective::Dual),
                None,
            ));
            radio(ui, &mut conf.direction, direction, color);
            space(ui);
        }

        space(ui);

        if let Some(edgepair) = conf.selected {
            if let Some(Some(sol)) = solution.next[conf.direction as usize].get(&edgepair) {
                ui.label("DUAL[");
                if sol.dual.is_ok() {
                    ui.label(colored_text("Ok", OK_GREEN));
                } else {
                    ui.label(colored_text(
                        &format!("{:?}", sol.dual.as_ref().err()),
                        WARN_RED,
                    ));
                }
                ui.label("]");

                ui.label("EMBD[");
                if sol.layout.is_some() {
                    ui.label(colored_text("Ok", OK_GREEN));
                } else {
                    ui.label(colored_text("Not found", OK_GREEN));
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
}

/// The pipeline bar: one stage per phase, each with its actions and a stop
/// toggle in between.
fn pipeline_bar(
    ui: &mut Ui,
    jobs: &mut MessageWriter<'_, Job>,
    conf: &mut Configuration,
    solution: &SolutionResource,
    render_setting_store: &mut RenderObjectSettingStore,
    hover_preview: &mut HoverPreviewState,
) {
    let current = &solution.current_solution;
    let mut stopped = false;

    // Collect stage responses for hover/click detection.
    let mut stage_responses: Vec<(&'static str, bevy_egui::egui::InnerResponse<Option<()>>)> =
        Vec::new();

    // Input
    if let Some(resp) = stage(
        ui,
        stopped,
        "Input",
        current.mesh_ref.nr_verts() != 0,
        Status::Ok(format!("{}", current.mesh_ref.nr_verts())),
        |ui| {
            label(
                ui,
                &format!("{} vertices", current.mesh_ref.nr_verts()),
                REGULAR_TEXT_SIZE,
                TEXT_COLOR,
            );
        },
    ) {
        stage_responses.push(("Input", resp));
    }

    space(ui);
    space(ui);
    space(ui);
    space(ui);
    space(ui);
    space(ui);
    space(ui);

    // Field
    if let Some(resp) = stage(
        ui,
        stopped,
        "Field",
        current.mesh_ref.nr_verts() != 0,
        match current.fields {
            Some(_) => Status::Ok("Ok".to_string()),
            None => Status::Err("missing".to_string()),
        },
        |ui| {
            let params = &mut conf.fields_params;

            slider(ui, "iterations", &mut params.outer_iterations, 0..=100);
            slider(ui, "cg_iterations", &mut params.cg_iterations, 10..=500);

            space(ui);

            slider(ui, "smooth_weight", &mut params.smooth_weight, 0.0..=1.0);
            slider(ui, "axis_weight", &mut params.axis_weight, 0.0..=1.0);
            slider(ui, "coupling", &mut params.coupling_weight, 0.0..=0.2);

            space(ui);

            if sleek_button(ui, "default weights") {
                *params = FieldParams::default();
            }

            sep(ui);

            if sleek_button(ui, "compute") {
                jobs.write(Job::compute_fields(
                    solution.current_solution.clone(),
                    conf.clone(),
                ));
            }
        },
    ) {
        stage_responses.push(("Field", resp));
    }

    space(ui);
    space(ui);
    space(ui);

    // Graph
    if let Some(resp) = stage(
        ui,
        stopped,
        "Graph",
        current.fields.is_some(),
        match current.flow_graphs {
            Some(_) => Status::Ok("Ok".to_string()),
            None => Status::Err("missing".to_string()),
        },
        |ui| {
            slider(
                ui,
                "flow graph top %",
                &mut conf.flow_graph_top_percent,
                0.0..=100.0,
            );

            space(ui);

            sep(ui);

            if sleek_button(ui, "compute") {
                jobs.write(Job::compute_graph(
                    solution.current_solution.clone(),
                    conf.clone(),
                ));
            }
        },
    ) {
        stage_responses.push(("Graph", resp));
    }

    space(ui);
    space(ui);
    space(ui);
    space(ui);
    space(ui);
    space(ui);
    space(ui);

    // Loops
    if let Some(resp) = stage(
        ui,
        stopped,
        "Loops",
        current.flow_graphs.is_some(),
        Status::Ok(format!("{}", current.loops.len())),
        |ui| {
            if sleek_button(ui, "initialize") {
                jobs.write(Job::initialize_loops(current.clone(), conf.clone()));
            }
            if sleek_button(ui, "evolve") {
                jobs.write(Job::evolve(current.clone(), conf.clone()));
            }
            slider(ui, "iterations", &mut conf.iterations, 1..=20);
            slider(ui, "pool1", &mut conf.pool1, 1..=20);
            slider(ui, "pool2", &mut conf.pool2, 1..=50);
        },
    ) {
        stage_responses.push(("Loops", resp));
    }
    stop_toggle(ui, conf, &mut stopped, Phase::Loops);

    // Dual
    if let Some(resp) = stage(
        ui,
        stopped,
        "Dual",
        !current.loops.is_empty(),
        match &current.dual {
            Ok(_) => Status::Ok("Ok".to_string()),
            Err(err) => Status::Err(err.to_string()),
        },
        |ui| {
            if sleek_button(ui, "(re)compute") {
                jobs.write(Job::compute_dual(current.clone(), conf.clone()));
            }
        },
    ) {
        stage_responses.push(("Dual", resp));
    }
    stop_toggle(ui, conf, &mut stopped, Phase::Dual);

    // Layout

    if let Some(resp) = stage(
        ui,
        stopped,
        "Layout",
        current.dual.is_ok(),
        match &current.layout {
            Some(layout) => match (layout.alignment, layout.orthogonality) {
                (Some(alignment), Some(orthogonality)) => {
                    Status::Ok(format!("{alignment:.3}, {orthogonality:.3}"))
                }
                _ => Status::Err("missing".to_string()),
            },
            None => Status::Err("missing".to_string()),
        },
        |ui| {
            if sleek_button(ui, "(re)compute corners") {
                jobs.write(Job::place_corners(current.clone(), conf.clone()));
            }
            if sleek_button(ui, "optimize corners") {
                jobs.write(Job::smoothen_layout(current.clone(), conf.clone()));
            }
            space(ui);
            if sleek_button(ui, "(re)compute paths") {
                jobs.write(Job::place_paths(current.clone(), conf.clone()));
            }
            if sleek_button(ui, "optimize paths") {
                jobs.write(Job::path_straightening(current.clone(), conf.clone()));
            }
        },
    ) {
        stage_responses.push(("Layout", resp));
    }
    stop_toggle(ui, conf, &mut stopped, Phase::Layout);

    // Polycube
    if let Some(resp) = stage(
        ui,
        stopped,
        "Polycube",
        current.layout.is_some(),
        match current.polycube {
            Some(_) => Status::Ok("Ok".to_string()),
            None => Status::Err("missing".to_string()),
        },
        |ui| {
            ui.checkbox(&mut conf.unit, "unit");
            if sleek_button(ui, "(re)compute") {
                jobs.write(Job::compute_polycube(current.clone(), conf.clone()));
            }
        },
    ) {
        stage_responses.push(("Polycube", resp));
    }

    space(ui);
    space(ui);
    space(ui);
    space(ui);
    space(ui);
    space(ui);
    space(ui);

    // Quad
    #[cfg(feature = "quad")]
    if let Some(resp) = stage(
        ui,
        stopped,
        "Quad",
        current.polycube.is_some(),
        Status::Ok("".to_string()),
        |ui| {
            if sleek_button(ui, "(re)compute") {
                jobs.write(Job::compute_quad(current.clone(), conf.clone()));
            }
            slider(ui, "omega", &mut conf.omega, 1..=20);
        },
    ) {
        stage_responses.push(("Quad", resp));
    }

    space(ui);
    space(ui);
    space(ui);
    space(ui);

    // Hex
    #[cfg(feature = "hex")]
    if let Some(resp) = stage(
        ui,
        stopped,
        "Hex",
        current.polycube.is_some(),
        Status::Ok("".to_string()),
        |ui| {
            if sleek_button(ui, "(re)compute") {
                jobs.write(Job::compute_hex(current.clone(), conf.clone()));
            }
        },
    ) {
        stage_responses.push(("Hex", resp));
    }

    // ── Hover‑preview for pipeline stages ─────────────────────────
    let mut hovered_this_frame: Option<&'static str> = None;

    for (name, resp) in &stage_responses {
        if resp.response.clicked() {
            // Commit: apply permanently and clear hover state.
            if let Some(p) = find_preset(name) {
                apply_render_preset(render_setting_store, p);
            }
            hover_preview.active = Some(name);
            hover_preview.hovered = None;
            hover_preview.saved.clear();
        } else if resp.response.hovered() {
            hovered_this_frame = Some(name);
        }
    }

    // Handle hover enter / exit.
    if hovered_this_frame != hover_preview.hovered {
        if hover_preview.hovered.is_some() && !hover_preview.saved.is_empty() {
            restore_settings(render_setting_store, hover_preview.saved.clone());
            hover_preview.saved.clear();
        }
        if let Some(name) = hovered_this_frame {
            hover_preview.saved = save_settings(render_setting_store);
            if let Some(p) = find_preset(name) {
                apply_render_preset(render_setting_store, p);
            }
        }
        hover_preview.hovered = hovered_this_frame;
    }
}
