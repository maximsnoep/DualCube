use crate::{elements::PrincipalDirection, ActionEvent, Configuration, DrawLoopType, InputResource, SolutionResource};
use bevy::prelude::*;
use bevy_egui::egui::{emath::Numeric, Color32, RichText, Slider, TopBottomPanel, Ui};

pub fn ui(
    mut egui_ctx: bevy_egui::EguiContexts,
    mut ev_w: EventWriter<ActionEvent>,
    mut conf: ResMut<Configuration>,
    mut mesh_resmut: ResMut<InputResource>,
    mut solution: ResMut<SolutionResource>,
) {
    TopBottomPanel::top("panel").show(egui_ctx.ctx_mut(), |ui| {
        sep(ui);

        ui.horizontal(|ui| {
            ui.add_space(50.);

            ui.vertical(|ui| {
                ui.horizontal(|ui| {
                    if ui.button("Export").clicked() {
                        ev_w.send(ActionEvent::ExportState);
                    };

                    if ui.button("Load file").clicked() {
                        if let Some(path) = rfd::FileDialog::new().add_filter("triangulated geometry", &["obj", "stl", "save"]).pick_file() {
                            ev_w.send(ActionEvent::LoadFile(path));
                        }
                    }

                    if mesh_resmut.properties.source.is_empty() {
                        ui.label("No file loaded.");
                    } else {
                        ui.add_space(15.);
                        ui.label(mesh_resmut.properties.source.to_string());
                    }
                });

                ui.add_space(15.);

                ui.horizontal(|ui| {
                    let vert_color = if mesh_resmut.properties.nr_of_vertices >= 100_000 {
                        Color32::RED
                    } else if mesh_resmut.properties.nr_of_vertices >= 50_000 {
                        Color32::YELLOW
                    } else {
                        Color32::GREEN
                    };

                    ui.vertical(|ui| {
                        ui.label("VERTS");
                        if mesh_resmut.properties.nr_of_vertices > 0 {
                            ui.label(RichText::new(format!("{:}", mesh_resmut.properties.nr_of_vertices)).color(vert_color));
                        } else {
                            ui.label("-");
                        }
                    });

                    ui.add_space(5.);

                    ui.vertical(|ui| {
                        ui.label("EDGES");
                        if mesh_resmut.properties.nr_of_edges > 0 {
                            ui.label(RichText::new(format!("{:}", mesh_resmut.properties.nr_of_edges)).color(vert_color));
                        } else {
                            ui.label("-");
                        }
                    });

                    ui.add_space(5.);

                    ui.vertical(|ui| {
                        ui.label("FACES");
                        if mesh_resmut.properties.nr_of_faces > 0 {
                            ui.label(RichText::new(format!("{:}", mesh_resmut.properties.nr_of_faces)).color(vert_color));
                        } else {
                            ui.label("-");
                        }
                    });

                    ui.add_space(15.);

                    ui.vertical(|ui| {
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

                    ui.vertical(|ui| {
                        ui.label("STATUS");
                        if let Err(err) = &solution.primal {
                            warning(ui, &format!("ERROR: {err:?}"));
                        } else {
                            okido(ui, "OK");
                        }
                    });

                    ui.add_space(15.);

                    ui.vertical(|ui| {
                        if let Some(selected_edge) = conf.cur_selected {
                            if let Some(sol) = solution.next.get(&selected_edge) {
                                ui.label("SELECTED");
                                if let Some((_, Err(err))) = sol {
                                    warning(ui, &format!("ERROR: {err:?}"));
                                } else {
                                    okido(ui, "OK");
                                }
                            }
                        }
                    });

                    // ui.label(format!("Selected: {:?}", conf.cur_selected));

                    // ui.add_space(15.);

                    ui.vertical(|ui| {
                        ui.checkbox(&mut conf.draw_wireframe, "graph");
                        ui.checkbox(&mut conf.draw_wireframe_granny, "granny");
                        ui.checkbox(&mut conf.draw_vertices, "vertices");
                        ui.checkbox(&mut conf.draw_normals, "normals");

                        // if ui.button("test").clicked() {
                        //     for _ in 0..100 {
                        //         if let Some(granny) = &mut solution.dual.granulated_mesh {
                        //             let face_id = granny.random_faces(1)[0];
                        //             let (new_v_id, _) = granny.split_face(face_id);
                        //             let new_pos = granny.centroid(face_id);
                        //             granny.verts[new_v_id].set_position(new_pos);
                        //         }
                        //     }
                        // }
                    });

                    // ui.add_space(15.);

                    // ui.vertical(|ui| {
                    //     for render_type in [RenderType::Original, RenderType::Polycube] {
                    //         if radio(ui, &mut conf.render_type, render_type) {
                    //             mesh_resmut.as_mut();
                    //         }
                    //     }
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
                });

                ui.add_space(10.);

                ui.horizontal(|ui| {
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
                });

                ui.add_space(10.);

                ui.horizontal(|ui| {
                    if ui.button("SWAP VIEW").clicked() {
                        conf.swap_cameras = !conf.swap_cameras;
                    }

                    ui.add_space(15.);

                    ui.checkbox(&mut conf.black, "BLACK");
                });

                ui.add_space(5.);
            });
        });

        sep(ui);
    });
}

pub fn sep(ui: &mut Ui) {
    ui.add_space(10.);
    ui.separator();
    ui.add_space(10.);
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
