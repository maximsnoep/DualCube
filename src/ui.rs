use crate::{dual::PrincipalDirection, ActionEvent, ColorMode, Configuration, InputResource, SolutionResource};
use bevy::prelude::*;
use bevy_egui::egui::{emath::Numeric, text::LayoutJob, Align, Color32, FontId, Layout, Slider, TextFormat, TopBottomPanel, Ui};

pub fn setup(mut ui: bevy_egui::EguiContexts) {
    // Font
    let mut fonts = bevy_egui::egui::FontDefinitions::default();
    fonts.font_data.insert(
        "BerkeleyMonoTrial".to_owned(),
        bevy_egui::egui::FontData::from_static(include_bytes!("../assets/BerkeleyMonoTrial-Regular.ttf")),
    );
    fonts
        .families
        .entry(bevy_egui::egui::FontFamily::Proportional)
        .or_default()
        .insert(0, "BerkeleyMonoTrial".to_owned());
    fonts
        .families
        .entry(bevy_egui::egui::FontFamily::Monospace)
        .or_default()
        .insert(0, "BerkeleyMonoTrial".to_owned());
    ui.ctx_mut().set_fonts(fonts);

    // Theme
    ui.ctx_mut().set_visuals(bevy_egui::egui::Visuals::dark());
}

pub fn update(
    mut egui_ctx: bevy_egui::EguiContexts,
    mut ev_w: EventWriter<ActionEvent>,
    mut conf: ResMut<Configuration>,
    mut mesh_resmut: ResMut<InputResource>,
    solution: Res<SolutionResource>,
) {
    TopBottomPanel::top("panel").show_separator_line(false).show(egui_ctx.ctx_mut(), |ui| {
        ui.add_space(10.);

        ui.horizontal(|ui| {
            ui.with_layout(Layout::top_down(Align::TOP), |ui| {
                // FIRST ROW
                ui.with_layout(Layout::left_to_right(Align::TOP), |ui| {
                    // LEFT SIDE
                    ui.with_layout(Layout::left_to_right(Align::TOP), |ui| {
                        ui.add_space(15.);

                        if ui.button(text("Export")).clicked() {
                            ev_w.send(ActionEvent::ExportState);
                        };

                        ui.add_space(15.);

                        if ui.button(text("Load")).clicked() {
                            if let Some(path) = rfd::FileDialog::new().add_filter("triangulated geometry", &["obj", "stl", "save"]).pick_file() {
                                ev_w.send(ActionEvent::LoadFile(path));
                            }
                        }

                        ui.add_space(15.);

                        if mesh_resmut.properties.source.is_empty() {
                            ui.label(colored_text("No file loaded.", Color32::RED));
                        } else {
                            ui.add_space(15.);
                            ui.label(text(&mesh_resmut.properties.source.to_string()));
                        }
                    });

                    // RIGHT SIDE
                    ui.with_layout(Layout::right_to_left(Align::TOP), |ui| {
                        ui.add_space(15.);

                        let size = 13.0;

                        let mut job = LayoutJob::default();
                        job.append("FPS[", 0.0, text_format(size, Color32::WHITE));
                        let fps_color = if conf.fps >= 50.0 {
                            Color32::from_rgb(0, 255, 0)
                        } else if conf.fps >= 30.0 {
                            let r = ((1.0 - (conf.fps - 60.0) / (120.0 - 60.0)) * 255.) as u8;
                            Color32::from_rgb(r, 255, 0)
                        } else if conf.fps >= 15.0 {
                            let g = (((conf.fps - 30.0) / (60.0 - 30.0)) * 255.) as u8;
                            Color32::from_rgb(255, g, 0)
                        } else {
                            Color32::from_rgb(255, 0, 0)
                        };
                        job.append(&format!("{:.0}", conf.fps), 0.0, text_format(size, fps_color));
                        job.append("]", 0.0, text_format(size, Color32::WHITE));
                        ui.label(job);

                        ui.add_space(15.);

                        let sol = &solution.current_solution;
                        let mut job = LayoutJob::default();
                        job.append("DUAL[", 0.0, text_format(size, Color32::WHITE));
                        if sol.dual.is_ok() {
                            job.append("OK", 0.0, text_format(size, Color32::GREEN));
                        } else {
                            job.append(&format!("{:?}", sol.dual.as_ref().err()), 0.0, text_format(size, Color32::RED));
                        }
                        job.append("]    LAYOUT[", 0.0, text_format(size, Color32::WHITE));
                        if let Some(layout) = &sol.layout {
                            if layout.is_ok() {
                                job.append("OK", 0.0, text_format(size, Color32::GREEN));
                            } else {
                                job.append(&format!("{:?}", layout.as_ref().err()), 0.0, text_format(size, Color32::RED));
                            }
                        } else {
                            job.append("None", 0.0, text_format(size, Color32::RED));
                        }
                        job.append("]    ALIGNMENT [", 0.0, text_format(size, Color32::WHITE));
                        if let Some(alignment) = sol.alignment {
                            let alignment_color = if alignment >= 0.95 {
                                Color32::GREEN
                            } else if alignment >= 0.75 {
                                Color32::YELLOW
                            } else {
                                Color32::RED
                            };
                            job.append(&format!("{alignment:.3}"), 0.0, text_format(size, alignment_color));
                        } else {
                            job.append("None", 0.0, text_format(size, Color32::RED));
                        }
                        job.append("]    ORTHOGONALITY [", 0.0, text_format(size, Color32::WHITE));
                        if let Some(orthogonality) = sol.orthogonality {
                            let orthogonality_color = if orthogonality >= 0.95 {
                                Color32::GREEN
                            } else if orthogonality >= 0.75 {
                                Color32::YELLOW
                            } else {
                                Color32::RED
                            };
                            job.append(&format!("{orthogonality:.3}"), 0.0, text_format(size, orthogonality_color));
                        } else {
                            job.append("None", 0.0, text_format(size, Color32::RED));
                        }
                        job.append("]", 0.0, text_format(size, Color32::WHITE));

                        ui.label(job);
                    });
                });

                ui.add_space(15.);

                // SECOND ROW
                ui.with_layout(Layout::left_to_right(Align::TOP), |ui| {
                    // LEFT SIDE
                    ui.with_layout(Layout::left_to_right(Align::TOP), |ui| {
                        ui.add_space(15.);

                        ui.spacing_mut().slider_width = 30.0;

                        // slider(ui, "α", &mut conf.alpha, 1..=20);

                        // ui.add_space(15.);

                        // slider(ui, "β", &mut conf.beta, 1..=20);

                        // ui.add_space(15.);

                        if ui.button(text("SOLVE")).on_hover_text("Solve the current mesh.").clicked() {
                            ev_w.send(ActionEvent::Mutate);
                        }

                        ui.add_space(15.);

                        for color_mode in [ColorMode::Default, ColorMode::Alignment, ColorMode::Orthogonality] {
                            if radio(ui, &mut conf.color_mode, color_mode) {
                                mesh_resmut.as_mut();
                            }
                        }

                        ui.add_space(15.);
                    });

                    // RIGHT SIDE
                    ui.with_layout(Layout::right_to_left(Align::BOTTOM), |ui| {
                        ui.add_space(15.);

                        let mut job = LayoutJob::default();
                        job.append("Fm[", 0.0, text_format(13.0, Color32::WHITE));
                        if mesh_resmut.mesh.nr_faces() > 0 {
                            job.append(&format!("{:}", mesh_resmut.mesh.nr_faces()), 0.0, text_format(13.0, Color32::WHITE));
                        } else {
                            job.append("-", 0.0, text_format(13.0, Color32::GRAY));
                        }
                        job.append("]", 0.0, text_format(13.0, Color32::WHITE));
                        ui.label(job);

                        ui.add_space(5.);

                        let mut job = LayoutJob::default();
                        job.append("Em[", 0.0, text_format(13.0, Color32::WHITE));
                        if mesh_resmut.mesh.nr_edges() > 0 {
                            job.append(&format!("{:}", mesh_resmut.mesh.nr_edges() / 2), 0.0, text_format(13.0, Color32::WHITE));
                        } else {
                            job.append("-", 0.0, text_format(13.0, Color32::GRAY));
                        }
                        job.append("]", 0.0, text_format(13.0, Color32::WHITE));
                        ui.label(job);

                        ui.add_space(5.);

                        let mut job = LayoutJob::default();
                        job.append("Vm[", 0.0, text_format(13.0, Color32::WHITE));
                        if mesh_resmut.mesh.nr_verts() > 0 {
                            job.append(&format!("{:}", mesh_resmut.mesh.nr_verts()), 0.0, text_format(13.0, Color32::WHITE));
                        } else {
                            job.append("-", 0.0, text_format(13.0, Color32::GRAY));
                        }
                        job.append("]", 0.0, text_format(13.0, Color32::WHITE));
                        ui.label(job);

                        ui.add_space(5.);

                        if let Some(polycube) = &solution.current_solution.polycube {
                            let mut job = LayoutJob::default();
                            job.append("Fp[", 0.0, text_format(13.0, Color32::WHITE));
                            if polycube.structure.nr_faces() > 0 {
                                job.append(&format!("{:}", polycube.structure.nr_faces()), 0.0, text_format(13.0, Color32::WHITE));
                            } else {
                                job.append("-", 0.0, text_format(13.0, Color32::GRAY));
                            }
                            job.append("]", 0.0, text_format(13.0, Color32::WHITE));
                            ui.label(job);

                            ui.add_space(5.);

                            let mut job = LayoutJob::default();
                            job.append("Ep[", 0.0, text_format(13.0, Color32::WHITE));
                            if polycube.structure.nr_edges() > 0 {
                                job.append(&format!("{:}", polycube.structure.nr_edges() / 2), 0.0, text_format(13.0, Color32::WHITE));
                            } else {
                                job.append("-", 0.0, text_format(13.0, Color32::GRAY));
                            }
                            job.append("]", 0.0, text_format(13.0, Color32::WHITE));
                            ui.label(job);

                            ui.add_space(5.);

                            let mut job = LayoutJob::default();
                            job.append("Vp[", 0.0, text_format(13.0, Color32::WHITE));
                            if polycube.structure.nr_verts() > 0 {
                                job.append(&format!("{:}", polycube.structure.nr_verts()), 0.0, text_format(13.0, Color32::WHITE));
                            } else {
                                job.append("-", 0.0, text_format(13.0, Color32::GRAY));
                            }
                            job.append("]", 0.0, text_format(13.0, Color32::WHITE));
                            ui.label(job);
                        }
                    });
                });

                ui.add_space(15.);

                // THIRD ROW
                ui.with_layout(Layout::left_to_right(Align::TOP), |ui| {
                    // LEFT SIDE
                    ui.with_layout(Layout::left_to_right(Align::TOP), |ui| {
                        ui.add_space(15.);

                        if ui.button(text("SWAP CAMS")).clicked() {
                            conf.swap_cameras = !conf.swap_cameras;
                        }

                        ui.add_space(15.);

                        // slider(ui, "CAM SPEED", &mut conf.camera_speed, 0.0..=5.);

                        // ui.add_space(15.);

                        // if ui.button(text("RESET CAMS")).clicked() {
                        //     ev_w.send(ActionEvent::ResetCamera);
                        // }

                        // ui.add_space(15.);

                        if ui.checkbox(&mut conf.black, text("DUAL MODE")).changed() {
                            mesh_resmut.as_mut();
                        }

                        ui.add_space(15.);

                        // ui.checkbox(&mut conf.compute_primal, text("AUTOMATIC LAYOUT"));

                        // ui.add_space(15.);

                        ui.checkbox(&mut conf.interactive, text("INTERACTIVE MODE"));

                        ui.add_space(15.);

                        for direction in [PrincipalDirection::X, PrincipalDirection::Y, PrincipalDirection::Z] {
                            radio(ui, &mut conf.direction, direction);
                        }

                        ui.add_space(15.);

                        ui.checkbox(&mut conf.delete_mode, text("DELETE MODE"));

                        ui.add_space(15.);
                    });

                    // RIGHT SIDE
                    ui.with_layout(Layout::right_to_left(Align::BOTTOM), |ui| {});
                });

                ui.add_space(15.);

                // FOURTH ROW
                ui.with_layout(Layout::left_to_right(Align::TOP), |ui| {
                    ui.with_layout(Layout::right_to_left(Align::TOP), |ui| {
                        ui.add_space(15.);
                        ui.with_layout(Layout::top_down(Align::BOTTOM), |ui| {
                            ui.label(text("SELECTED"));
                            if let Some(edgepair) = conf.selected {
                                if let Some(Some(sol)) = solution.next[conf.direction as usize].get(&edgepair) {
                                    let size = 13.0;
                                    let mut job = LayoutJob::default();
                                    job.append("DUAL[", 0.0, text_format(size, Color32::WHITE));
                                    if sol.dual.is_ok() {
                                        job.append("OK", 0.0, text_format(size, Color32::GREEN));
                                    } else {
                                        job.append(&format!("{:?}", sol.dual.as_ref().err()), 0.0, text_format(size, Color32::RED));
                                    }
                                    job.append("]    LAYOUT[", 0.0, text_format(size, Color32::WHITE));
                                    if let Some(layout) = &sol.layout {
                                        if layout.is_ok() {
                                            job.append("OK", 0.0, text_format(size, Color32::GREEN));
                                        } else {
                                            job.append(&format!("{:?}", layout.as_ref().err()), 0.0, text_format(size, Color32::RED));
                                        }
                                    } else {
                                        job.append("None", 0.0, text_format(size, Color32::RED));
                                    }
                                    job.append("]    ALIGNMENT [", 0.0, text_format(size, Color32::WHITE));
                                    if let Some(alignment) = sol.alignment {
                                        let alignment_color = if alignment >= 0.95 {
                                            Color32::GREEN
                                        } else if alignment >= 0.75 {
                                            Color32::YELLOW
                                        } else {
                                            Color32::RED
                                        };
                                        job.append(&format!("{alignment:.3}"), 0.0, text_format(size, alignment_color));
                                    } else {
                                        job.append("None", 0.0, text_format(size, Color32::RED));
                                    }
                                    job.append("]    ORTHOGONALITY [", 0.0, text_format(size, Color32::WHITE));
                                    if let Some(orthogonality) = sol.orthogonality {
                                        let orthogonality_color = if orthogonality >= 0.95 {
                                            Color32::GREEN
                                        } else if orthogonality >= 0.75 {
                                            Color32::YELLOW
                                        } else {
                                            Color32::RED
                                        };
                                        job.append(&format!("{orthogonality:.3}"), 0.0, text_format(size, orthogonality_color));
                                    } else {
                                        job.append("None", 0.0, text_format(size, Color32::RED));
                                    }
                                    job.append("]", 0.0, text_format(size, Color32::WHITE));

                                    ui.label(job);
                                } else {
                                    ui.label(colored_text("-", Color32::GRAY));
                                }
                            } else {
                                ui.label(colored_text("-", Color32::GRAY));
                            }
                        });

                        ui.add_space(15.);
                    });
                });

                ui.add_space(15.);
            });
        });
    });
}

fn slider<T: Numeric>(ui: &mut Ui, label: &str, value: &mut T, range: std::ops::RangeInclusive<T>) {
    ui.add(Slider::new(value, range).text(text(label)));
}

fn stepper(ui: &mut Ui, label: &str, value: &mut u32, min: u32, max: u32) -> bool {
    ui.horizontal(|ui| {
        if ui.button("<<").clicked() {
            let new_value = *value - 1;
            if new_value >= min && new_value <= max {
                *value = new_value;
            } else {
                *value = max;
            };
            return true;
        }
        ui.label(format!("{label}: {value} [{min}-{max}]"));
        if ui.button(">>").clicked() {
            let new_value = *value + 1;
            if new_value <= max && new_value >= min {
                *value = new_value;
            } else {
                *value = min;
            };
            return true;
        }
        false
    })
    .inner
}

fn radio<T: PartialEq<T> + std::fmt::Display>(ui: &mut Ui, item: &mut T, value: T) -> bool {
    if ui.radio(*item == value, text(&format!("{value}"))).clicked() {
        *item = value;
        true
    } else {
        false
    }
}

pub fn text(string: &str) -> LayoutJob {
    let mut job = LayoutJob::default();
    job.append(string, 0.0, text_format(13.0, Color32::WHITE));
    job
}

pub fn colored_text(string: &str, color: Color32) -> LayoutJob {
    let mut job = LayoutJob::default();
    job.append(string, 0.0, text_format(13.0, color));
    job
}

pub fn text_format(size: f32, color: Color32) -> TextFormat {
    TextFormat {
        font_id: FontId {
            size,
            family: bevy_egui::egui::FontFamily::Monospace,
        },
        color,
        ..Default::default()
    }
}
