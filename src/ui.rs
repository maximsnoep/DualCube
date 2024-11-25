use crate::{
    camera::{CameraFor, Objects},
    dual::PrincipalDirection,
    ActionEvent, CameraHandles, Configuration, InputResource, SolutionResource,
};
use bevy::prelude::*;
use bevy_egui::egui::Rounding;
use bevy_egui::egui::{emath::Numeric, text::LayoutJob, Align, Color32, FontId, Frame, Layout, Slider, TextFormat, TopBottomPanel, Ui, Window};
use tico::tico;

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

    ui.ctx_mut().style_mut(|style| {
        style.visuals.widgets.open.rounding = Rounding::same(0.);
        style.visuals.menu_rounding = Rounding::same(0.);
        style.visuals.window_rounding = Rounding::same(0.);
        style.visuals.widgets.noninteractive.rounding = Rounding::same(0.);
        style.visuals.widgets.hovered.rounding = Rounding::same(0.);
        style.visuals.widgets.active.rounding = Rounding::same(0.);
    });
}

pub fn update(
    mut egui_ctx: bevy_egui::EguiContexts,
    mut ev_w: EventWriter<ActionEvent>,
    mut conf: ResMut<Configuration>,
    mut mesh_resmut: ResMut<InputResource>,
    solution: Res<SolutionResource>,
    time: Res<Time>,
    image_handle: Res<CameraHandles>,
) {
    TopBottomPanel::top("panel").show_separator_line(false).show(egui_ctx.ctx_mut(), |ui| {
        ui.add_space(10.);

        ui.horizontal(|ui| {
            ui.with_layout(Layout::top_down(Align::TOP), |ui| {
                // FIRST ROW
                ui.with_layout(Layout::left_to_right(Align::TOP), |ui| {
                    Frame {
                        outer_margin: bevy_egui::egui::epaint::Margin::symmetric(15., 0.),
                        shadow: bevy_egui::egui::epaint::Shadow::NONE,
                        ..default()
                    }
                    .show(ui, |ui| {
                        bevy_egui::egui::menu::bar(ui, |ui| {
                            bevy_egui::egui::menu::menu_button(ui, "File", |ui| {
                                if ui.button("Export").clicked() {
                                    ev_w.send(ActionEvent::ExportState);
                                }
                                ui.add_space(5.);
                                if ui.button("Load").clicked() {
                                    if let Some(path) = rfd::FileDialog::new().add_filter("triangulated geometry", &["obj", "stl", "save"]).pick_file() {
                                        ev_w.send(ActionEvent::LoadFile(path));
                                    }
                                }
                                ui.add_space(5.);
                                ui.separator();
                                ui.add_space(5.);
                                if ui.button("Quit").clicked() {
                                    std::process::exit(0);
                                }
                            });

                            ui.separator();

                            bevy_egui::egui::menu::menu_button(ui, "Info", |ui| {
                                if mesh_resmut.properties.source.is_empty() {
                                    ui.label("No file loaded.");
                                } else {
                                    ui.label(tico(
                                        &mesh_resmut
                                            .properties
                                            .source
                                            .to_string()
                                            .chars()
                                            .map(|ch| if ch == '\\' { '/' } else { ch })
                                            .collect::<String>(),
                                        None,
                                    ));
                                    ui.add_space(5.);
                                    ui.label(format!(
                                        "|Vm|: {}\n|Em|: {}\n|Fm|: {}",
                                        mesh_resmut.mesh.nr_verts(),
                                        mesh_resmut.mesh.nr_edges() / 2,
                                        mesh_resmut.mesh.nr_faces()
                                    ));
                                }

                                if let Some(polycube) = &solution.current_solution.polycube {
                                    ui.add_space(5.);
                                    ui.label(format!(
                                        "|Vp|: {}\n|Ep|: {}\n|Fp|: {}",
                                        polycube.structure.nr_verts(),
                                        polycube.structure.nr_edges() / 2,
                                        polycube.structure.nr_faces()
                                    ));
                                }
                            });

                            ui.separator();

                            bevy_egui::egui::menu::menu_button(ui, "Controls", |ui| {
                                ui.label("CAMERA");
                                ui.add_space(2.);
                                ui.label("  Rotate: ctrl + right-mouse-drag");
                                ui.add_space(1.);
                                ui.label("  Pan: left-mouse-drag");
                                ui.add_space(1.);
                                ui.label("  Zoom: mouse-scroll");
                                ui.add_space(2.);
                                ui.separator();
                                ui.add_space(2.);
                                ui.label("MANUAL");
                                ui.add_space(2.);
                                ui.label("  Add loop: right-mouse-click");
                                ui.add_space(1.);
                                ui.label("  Delete loop: space + right-mouse-click");
                            });
                        });
                    });
                });

                ui.add_space(5.);

                ui.separator();

                ui.add_space(5.);

                // SECOND ROW
                ui.with_layout(Layout::left_to_right(Align::TOP), |ui| {
                    ui.add_space(15.);
                    if ui.button("SMOOTHEN").clicked() {
                        ev_w.send(ActionEvent::Smoothen);
                    }
                });

                ui.add_space(5.);

                // SECOND ROW
                ui.with_layout(Layout::left_to_right(Align::TOP), |ui| {
                    ui.add_space(15.);
                    if ui.checkbox(&mut conf.should_continue, "AUTO").clicked() && conf.should_continue {
                        ev_w.send(ActionEvent::Mutate);
                        conf.interactive = false;
                    }
                });

                ui.add_space(5.);

                // THIRD ROW
                ui.with_layout(Layout::left_to_right(Align::TOP), |ui| {
                    ui.add_space(15.);
                    if ui.checkbox(&mut conf.interactive, "MANUAL").clicked() {
                        conf.should_continue = false;
                    };
                    ui.add_space(15.);
                    if conf.interactive {
                        for direction in [PrincipalDirection::X, PrincipalDirection::Y, PrincipalDirection::Z] {
                            radio(
                                ui,
                                &mut conf.direction,
                                direction,
                                Color32::from_rgb(
                                    (direction.to_dual_color()[0] * 255.) as u8,
                                    (direction.to_dual_color()[1] * 255.) as u8,
                                    (direction.to_dual_color()[2] * 255.) as u8,
                                ),
                            );
                        }

                        if let Some(edgepair) = conf.selected {
                            if let Some(Some(sol)) = solution.next[conf.direction as usize].get(&edgepair) {
                                ui.label("DUAL[");
                                if sol.dual.is_ok() {
                                    ui.label(colored_text("Ok", Color32::GREEN));
                                } else {
                                    ui.label(colored_text(&format!("{:?}", sol.dual.as_ref().err()), Color32::RED));
                                }
                                ui.label("]");

                                ui.label("EMBD[");
                                if let Some(layout) = &sol.layout {
                                    if layout.is_ok() {
                                        ui.label(colored_text("Ok", Color32::GREEN));
                                    } else {
                                        ui.label(colored_text(&format!("{:?}", layout.as_ref().err()), Color32::RED));
                                    }
                                } else {
                                    ui.label(colored_text("None", Color32::RED));
                                }
                                ui.label("]");

                                if let Some(alignment) = sol.alignment {
                                    ui.label("ALIGN[");
                                    ui.label(format!("{alignment:.3}"));
                                    ui.label("]");
                                }

                                if let Some(orthogonality) = sol.orthogonality {
                                    ui.label("ORTH[");
                                    ui.label(format!("{orthogonality:.3}"));
                                    ui.label("]");
                                }
                            }
                        }
                    }
                });
            });
        });

        conf.ui_is_hovered[0] = ui.ui_contains_pointer();
    });

    TopBottomPanel::bottom("footer").show_separator_line(false).show(egui_ctx.ctx_mut(), |ui| {
        ui.horizontal(|ui| {
            ui.with_layout(Layout::left_to_right(Align::TOP), |ui| {
                // LEFT SIDE
                ui.with_layout(Layout::left_to_right(Align::TOP), |ui| {
                    ui.add_space(15.);

                    let mut job = LayoutJob::default();
                    job.append(&format!("{:.0}", conf.fps), 0.0, text_format(9.0, Color32::WHITE));
                    ui.label(job);

                    ui.add_space(5.);

                    let mut job = LayoutJob::default();
                    let elapsed = (time.elapsed_seconds() / 4.) % 1.;
                    let label = if elapsed < 0.25 {
                        " . "
                    } else if elapsed < 0.5 {
                        ".. "
                    } else if elapsed < 0.75 {
                        "..."
                    } else {
                        " .."
                    };
                    job.append(label, 0.0, text_format(9.0, Color32::WHITE));
                    ui.label(job);
                });

                // CENTER
                ui.vertical_centered(|ui| {
                    let mut job = LayoutJob::default();
                    job.append("dual", 0.0, text_format(9.0, Color32::WHITE));
                    job.append("[", 0.0, text_format(9.0, Color32::WHITE));
                    match &solution.current_solution.dual {
                        Ok(_) => job.append("Ok", 0.0, text_format(9.0, Color32::GREEN)),
                        Err(err) => job.append(&format!("{:?}", err), 0.0, text_format(9.0, Color32::RED)),
                    }
                    job.append("]", 0.0, text_format(9.0, Color32::WHITE));
                    job.append("embd", 5.0, text_format(9.0, Color32::WHITE));
                    job.append("[", 0.0, text_format(9.0, Color32::WHITE));
                    match &solution.current_solution.layout {
                        Some(Ok(_)) => job.append("Ok", 0.0, text_format(9.0, Color32::GREEN)),
                        Some(Err(err)) => job.append(&format!("{:?}", err), 0.0, text_format(9.0, Color32::RED)),
                        None => job.append("None", 0.0, text_format(9.0, Color32::RED)),
                    }
                    job.append("]", 0.0, text_format(9.0, Color32::WHITE));
                    if let Some(alignment) = solution.current_solution.alignment {
                        job.append("align", 5.0, text_format(9.0, Color32::WHITE));
                        job.append(&format!("[{alignment:.3}]"), 0.0, text_format(9.0, Color32::WHITE));
                    }
                    if let Some(orthogonality) = solution.current_solution.orthogonality {
                        job.append("orth", 5.0, text_format(9.0, Color32::WHITE));
                        job.append(&format!("[{orthogonality:.3}]"), 0.0, text_format(9.0, Color32::WHITE));
                    }
                    ui.label(job);
                });

                // RIGHT SIDE
                ui.with_layout(Layout::right_to_left(Align::TOP), |ui| {
                    ui.add_space(15.);

                    let mut job = LayoutJob::default();
                    job.append("maxim snoep", 0.0, text_format(9.0, Color32::WHITE));
                    ui.label(job);
                });
            });
        });

        conf.ui_is_hovered[1] = ui.ui_contains_pointer();
    });

    bevy_egui::egui::CentralPanel::default()
        .frame(Frame {
            outer_margin: bevy_egui::egui::epaint::Margin::same(10.),
            stroke: bevy_egui::egui::epaint::Stroke {
                width: 10.0,
                color: Color32::from_rgb(27, 27, 27),
            },
            shadow: bevy_egui::egui::epaint::Shadow::NONE,
            ..default()
        })
        .show(egui_ctx.ctx_mut(), |ui| {
            Frame {
                outer_margin: bevy_egui::egui::epaint::Margin::same(1.),
                stroke: bevy_egui::egui::epaint::Stroke {
                    width: 1.0,
                    color: Color32::from_rgb(50, 50, 50),
                },
                shadow: bevy_egui::egui::epaint::Shadow::NONE,
                ..default()
            }
            .show(ui, |ui| {
                ui.allocate_space(ui.available_size());
            });
        });

    for i in 0..conf.window_shows_object.len() {
        let egui_handle = egui_ctx.add_image(image_handle.map.get(&CameraFor(conf.window_shows_object[i])).unwrap().clone());
        Window::new(format!("window {i}"))
            .frame(Frame {
                outer_margin: bevy_egui::egui::epaint::Margin::same(2.),
                stroke: bevy_egui::egui::epaint::Stroke {
                    width: 1.0,
                    color: Color32::from_rgb(50, 50, 50),
                },
                shadow: bevy_egui::egui::epaint::Shadow::NONE,
                fill: Color32::from_rgb(27, 27, 27),
                ..default()
            })
            .max_size([conf.window_has_size[i], conf.window_has_size[i]])
            .title_bar(false)
            .id(format!("window {i}").into())
            .resizable([false, false])
            .show(egui_ctx.ctx_mut(), |ui| {
                Frame {
                    outer_margin: bevy_egui::egui::epaint::Margin::symmetric(15., 0.),
                    shadow: bevy_egui::egui::epaint::Shadow::NONE,
                    ..default()
                }
                .show(ui, |ui| {
                    bevy_egui::egui::menu::bar(ui, |ui| {
                        bevy_egui::egui::menu::menu_button(ui, String::from(conf.window_shows_object[i]), |ui| {
                            for object in [
                                Objects::PolycubeDual,
                                Objects::PolycubePrimal,
                                Objects::MeshPolycubeLayout,
                                Objects::MeshInput,
                                Objects::MeshAlignmentScore,
                                Objects::MeshOrthogonalityScore,
                            ] {
                                if ui.button(String::from(object)).clicked() {
                                    conf.window_shows_object[i] = object;
                                }
                            }
                        });

                        if ui.button("+").clicked() {
                            if conf.window_has_size[i] > 0. {
                                conf.window_has_size[i] += 64.;
                                if conf.window_has_size[i] > 640. {
                                    conf.window_has_size[i] = 640.;
                                }
                            } else {
                                conf.window_has_size[i] = 256.;
                            }
                        }
                        if ui.button("-").clicked() {
                            conf.window_has_size[i] -= 64.;
                            if conf.window_has_size[i] < 256. {
                                conf.window_has_size[i] = 0.;
                            }
                        }
                    });
                });

                ui.add(bevy_egui::egui::widgets::Image::new(bevy_egui::egui::load::SizedTexture::new(
                    egui_handle,
                    [conf.window_has_size[i], conf.window_has_size[i]],
                )));

                conf.ui_is_hovered[31 - i] = ui.ui_contains_pointer() || ui.ctx().is_being_dragged(format!("window {i}").into());
            });
    }
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

fn radio<T: PartialEq<T> + std::fmt::Display>(ui: &mut Ui, item: &mut T, value: T, color: Color32) -> bool {
    if ui.radio(*item == value, colored_text(&format!("{value}"), color)).clicked() {
        *item = value;
        true
    } else {
        false
    }
}

pub fn text(string: &str) -> LayoutJob {
    colored_text(string, Color32::WHITE)
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
