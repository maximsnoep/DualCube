use crate::{dual::PrincipalDirection, ActionEvent, Configuration, InputResource, SolutionResource};
use bevy::prelude::*;
use bevy_egui::egui::{emath::Numeric, Align, Color32, Layout, RichText, Slider, TopBottomPanel, Ui};

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
        .push("BerkeleyMonoTrial".to_owned());
    ui.ctx_mut().set_fonts(fonts);

    // Theme
    ui.ctx_mut().set_visuals(bevy_egui::egui::Visuals::light());
}

pub fn update(
    mut egui_ctx: bevy_egui::EguiContexts,
    mut ev_w: EventWriter<ActionEvent>,
    mut conf: ResMut<Configuration>,
    mut mesh_resmut: ResMut<InputResource>,
    solution: Res<SolutionResource>,
) {
    TopBottomPanel::top("panel").show(egui_ctx.ctx_mut(), |ui| {
        ui.add_space(10.);

        ui.horizontal(|ui| {
            ui.with_layout(Layout::top_down(Align::TOP), |ui| {
                // FIRST ROW
                ui.with_layout(Layout::left_to_right(Align::TOP), |ui| {
                    // LEFT SIDE
                    ui.with_layout(Layout::left_to_right(Align::TOP), |ui| {
                        ui.add_space(15.);

                        if ui.button("Export").clicked() {
                            ev_w.send(ActionEvent::ExportState);
                        };

                        ui.add_space(15.);

                        if ui.button("Load file").clicked() {
                            if let Some(path) = rfd::FileDialog::new().add_filter("triangulated geometry", &["obj", "stl", "save"]).pick_file() {
                                ev_w.send(ActionEvent::LoadFile(path));
                            }
                        }

                        ui.add_space(15.);

                        if mesh_resmut.properties.source.is_empty() {
                            ui.label("No file loaded.");
                        } else {
                            ui.add_space(15.);
                            ui.label(mesh_resmut.properties.source.to_string());
                        }

                        ui.add_space(15.);
                    });

                    // RIGHT SIDE
                    ui.with_layout(Layout::right_to_left(Align::TOP), |ui| {
                        ui.add_space(15.);

                        ui.with_layout(Layout::top_down(Align::BOTTOM), |ui| {
                            ui.label("FPS");
                            if conf.fps > 0. {
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
                                ui.label(RichText::new(format!("{:.0}", conf.fps)).color(fps_color));
                            }
                        });

                        ui.add_space(15.);

                        ui.with_layout(Layout::top_down(Align::BOTTOM), |ui| {
                            ui.label("CURRENT STATUS");
                            if let Err(err) = &solution.primal {
                                warning(ui, &format!("ERROR: {err:?}"));
                            } else {
                                okido(ui, "OK");
                            }
                        });
                    });
                });

                ui.add_space(15.);

                // SECOND ROW
                ui.with_layout(Layout::left_to_right(Align::TOP), |ui| {
                    // LEFT SIDE
                    ui.with_layout(Layout::left_to_right(Align::TOP), |ui| {
                        ui.add_space(15.);

                        ui.checkbox(&mut conf.interactive, "interactive");

                        ui.add_space(15.);

                        ui.spacing_mut().slider_width = 30.0;

                        slider(ui, "α", &mut conf.alpha, 1..=20);

                        ui.add_space(15.);

                        slider(ui, "β", &mut conf.beta, 1..=20);

                        ui.add_space(15.);

                        for direction in [PrincipalDirection::X, PrincipalDirection::Y, PrincipalDirection::Z] {
                            radio(ui, &mut conf.direction, direction);
                        }

                        ui.add_space(15.);
                    });

                    // RIGHT SIDE
                    ui.with_layout(Layout::right_to_left(Align::BOTTOM), |ui| {
                        ui.add_space(15.);

                        ui.with_layout(Layout::top_down(Align::BOTTOM), |ui| {
                            ui.label("FACES");
                            if mesh_resmut.properties.nr_of_faces > 0 {
                                ui.label(format!("{:}", mesh_resmut.properties.nr_of_faces));
                            } else {
                                ui.label("-");
                            }
                        });

                        ui.add_space(5.);

                        ui.with_layout(Layout::top_down(Align::BOTTOM), |ui| {
                            ui.label("EDGES");
                            if mesh_resmut.properties.nr_of_edges > 0 {
                                ui.label(format!("{:}", mesh_resmut.properties.nr_of_edges));
                            } else {
                                ui.label("-");
                            }
                        });

                        ui.add_space(5.);

                        ui.with_layout(Layout::top_down(Align::BOTTOM), |ui| {
                            ui.label("VERTS");
                            if mesh_resmut.properties.nr_of_vertices > 0 {
                                ui.label(format!("{:}", mesh_resmut.properties.nr_of_vertices));
                            } else {
                                ui.label("-");
                            }
                        });

                        ui.add_space(15.);

                        ui.with_layout(Layout::top_down(Align::BOTTOM), |ui| {
                            ui.label("INPUT");
                        });
                    });
                });

                ui.add_space(15.);

                // THIRD ROW
                ui.with_layout(Layout::left_to_right(Align::TOP), |ui| {
                    // LEFT SIDE
                    ui.with_layout(Layout::left_to_right(Align::TOP), |ui| {
                        ui.add_space(15.);

                        if ui.button("Swap").clicked() {
                            conf.swap_cameras = !conf.swap_cameras;
                        }

                        ui.add_space(15.);

                        if ui.checkbox(&mut conf.black, "Dual").changed() {
                            mesh_resmut.as_mut();
                        }

                        ui.add_space(15.);
                    });

                    // RIGHT SIDE
                    ui.with_layout(Layout::right_to_left(Align::BOTTOM), |ui| {
                        ui.add_space(15.);

                        ui.with_layout(Layout::top_down(Align::BOTTOM), |ui| {
                            ui.label("FACES");
                            if solution.properties.nr_of_faces > 0 {
                                ui.label(format!("{:}", solution.properties.nr_of_faces));
                            } else {
                                ui.label("-");
                            }
                        });

                        ui.add_space(5.);

                        ui.with_layout(Layout::top_down(Align::BOTTOM), |ui| {
                            ui.label("EDGES");
                            if solution.properties.nr_of_edges > 0 {
                                ui.label(format!("{:}", solution.properties.nr_of_edges));
                            } else {
                                ui.label("-");
                            }
                        });

                        ui.add_space(5.);

                        ui.with_layout(Layout::top_down(Align::BOTTOM), |ui| {
                            ui.label("VERTS");
                            if solution.properties.nr_of_vertices > 0 {
                                ui.label(format!("{:}", solution.properties.nr_of_vertices));
                            } else {
                                ui.label("-");
                            }
                        });

                        ui.add_space(15.);

                        ui.with_layout(Layout::top_down(Align::BOTTOM), |ui| {
                            ui.label("POLYCUBE");
                        });
                    });
                });

                ui.add_space(15.);

                ui.with_layout(Layout::left_to_right(Align::TOP), |ui| {
                    ui.with_layout(Layout::right_to_left(Align::TOP), |ui| {
                        ui.add_space(15.);

                        // Selected face etc.
                        ui.with_layout(Layout::top_down(Align::BOTTOM), |ui| {
                            ui.label("RAYCASTED FACE");
                            if let Some(selected_face) = conf.selected_face {
                                ui.label(selected_face.0.to_string());
                            } else {
                                ui.label("-");
                            }

                            ui.add_space(15.);

                            ui.label("HIGHLIGHTED SOLUTION");

                            if let Some(selected_edge) = conf.selected_solution {
                                ui.label(format!("{}", selected_edge.0));
                            } else {
                                ui.label("-");
                            }

                            ui.label("STATUS");

                            if let Some(selected_edge) = conf.selected_solution {
                                if let Some(sol) = solution.next[conf.direction as usize].get(&selected_edge) {
                                    if let Some((_, Err(err))) = sol {
                                        warning(ui, &format!("ERROR: {err:?}"));
                                    } else {
                                        okido(ui, "OK");
                                    }
                                } else {
                                    ui.label("-");
                                }
                            } else {
                                ui.label("-");
                            }
                        });

                        ui.add_space(15.);
                    });
                });
                // ui.add_space(15.);

                // ui.vertical(|ui| {
                //     ui.checkbox(&mut conf.draw_wireframe, "graph");
                //     ui.checkbox(&mut conf.draw_vertices, "vertices");
                //     ui.checkbox(&mut conf.draw_normals, "normals");
                // });

                // ui.vertical(|ui| {
                //     ui.label("Sublabels");
                //     for i in 0..3 {
                //         if stepper(
                //             ui,
                //             ["X-loops", "Y-loops", "Z-loops"][i],
                //             &mut conf.sides_mask[i],
                //             0,
                //             2_u32.pow(u32::try_from(solution.dual.side_ccs[i].len()).unwrap()) - 1,
                //         ) {
                //             let res = solution.dual.zone_graph(conf.sides_mask);
                //             if let Err(err) = res {
                //                 solution.primal = Err(err);
                //             }
                //             solution.primal = Ok(solution.dual.primal());
                //             mesh_resmut.as_mut();
                //         }
                //     }
                // });

                ui.add_space(10.);
            });
        });
    });
}

fn slider<T: Numeric>(ui: &mut Ui, label: &str, value: &mut T, range: std::ops::RangeInclusive<T>) {
    ui.add(Slider::new(value, range).text(label));
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
    if ui.radio(*item == value, format!("{value}")).clicked() {
        *item = value;
        true
    } else {
        false
    }
}

fn warning(ui: &mut Ui, text: &str) {
    ui.label(RichText::new(text).color(Color32::RED));
}

fn okido(ui: &mut Ui, text: &str) {
    ui.label(RichText::new(text).color(Color32::GREEN));
}
