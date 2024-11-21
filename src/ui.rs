use crate::{
    camera::{CameraFor, Objects},
    dual::PrincipalDirection,
    ActionEvent, CameraHandles, ColorMode, Configuration, InputResource, SolutionResource,
};
use bevy::prelude::*;
use bevy_egui::egui::CursorIcon;
use bevy_egui::egui::{emath::Numeric, text::LayoutJob, Align, Color32, FontId, Frame, Layout, Slider, TextFormat, TopBottomPanel, Ui, Window};

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
    time: Res<Time>,
    image_handle: Res<CameraHandles>,
) {
    TopBottomPanel::top("panel").show_separator_line(false).show(egui_ctx.ctx_mut(), |ui| {
        ui.add_space(10.);

        ui.horizontal(|ui| {
            ui.with_layout(Layout::top_down(Align::TOP), |ui| {
                // FIRST ROW
                ui.with_layout(Layout::left_to_right(Align::TOP), |ui| {
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

                        bevy_egui::egui::menu::menu_button(ui, "Controls", |ui| {
                            ui.label("Left click: rotate");
                            ui.add_space(5.);
                            ui.label("Right click: pan");
                            ui.add_space(5.);
                            ui.label("Scroll: zoom");
                            ui.add_space(5.);
                            ui.label("Middle click: reset");
                        });

                        ui.separator();

                        bevy_egui::egui::menu::menu_button(ui, "Info", |ui| {
                            ui.label("Loaded file: ...");
                            ui.label("Number of vertices: ////");
                            ui.label("Scroll: zoom");
                            ui.label("Middle click: reset");
                        });
                    });
                });

                ui.add_space(15.);

                ui.separator();

                ui.add_space(15.);

                // SECOND ROW
                ui.with_layout(Layout::left_to_right(Align::TOP), |ui| {
                    // LEFT SIDE
                    ui.with_layout(Layout::left_to_right(Align::TOP), |ui| {
                        ui.spacing_mut().slider_width = 30.0;

                        // slider(ui, "α", &mut conf.alpha, 1..=20);

                        // ui.add_space(15.);

                        // slider(ui, "β", &mut conf.beta, 1..=20);

                        // ui.add_space(15.);

                        let size = 13.;

                        ui.add_space(15.);

                        if mesh_resmut.properties.source.is_empty() {
                            ui.label(colored_text("No file loaded.", Color32::RED));
                        } else {
                            ui.add_space(15.);
                            ui.label(text(&mesh_resmut.properties.source.to_string()));
                        }

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

                        if ui.checkbox(&mut conf.should_continue, text("AUTOMATIC")).clicked() && conf.should_continue {
                            ev_w.send(ActionEvent::Mutate);
                        }
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
                        ui.checkbox(&mut conf.interactive, text("MANUAL"));
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
                        } else {
                            for direction in [PrincipalDirection::X, PrincipalDirection::Y, PrincipalDirection::Z] {
                                disabled_radio(ui, &direction, Color32::GRAY);
                            }
                        }
                    });
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
                    color: Color32::DARK_GRAY,
                },
                shadow: bevy_egui::egui::epaint::Shadow::NONE,
                ..default()
            }
            .show(ui, |ui| {
                ui.allocate_space(ui.available_size());
            });
        });

    for i in 0..3 {
        let egui_handle = egui_ctx.add_image(image_handle.map.get(&CameraFor(conf.window_shows_object[i])).unwrap().clone());
        Window::new(format!("window {i}"))
            .frame(Frame {
                outer_margin: bevy_egui::egui::epaint::Margin::same(2.),
                stroke: bevy_egui::egui::epaint::Stroke {
                    width: 1.0,
                    color: Color32::DARK_GRAY,
                },
                shadow: bevy_egui::egui::epaint::Shadow::NONE,
                fill: Color32::from_rgb(27, 27, 27),
                ..default()
            })
            .title_bar(false)
            //.anchor(Align2::RIGHT_TOP, [-5.0, 100.0])
            .resizable([true, true])
            .show(egui_ctx.ctx_mut(), |ui| {
                bevy_egui::egui::ComboBox::from_label("")
                    .selected_text(text((conf.window_shows_object[i]).into()))
                    .show_ui(ui, |ui| {
                        for object in [
                            Objects::PolycubeDual,
                            Objects::PolycubePrimal,
                            Objects::MeshPolycubeLayout,
                            Objects::MeshInput,
                            Objects::MeshAlignmentScore,
                            Objects::MeshOrthogonalityScore,
                        ] {
                            ui.selectable_value(&mut conf.window_shows_object[i], object, text(object.into()));
                        }
                    });
                ui.add(bevy_egui::egui::widgets::Image::new(bevy_egui::egui::load::SizedTexture::new(
                    egui_handle,
                    [444.0, 444.0],
                )));
                let contains_pointer = ui.ui_contains_pointer();
                conf.ui_is_hovered[31 - i] = contains_pointer;
                if contains_pointer {
                    // change cursor to grabber icon:
                    ui.ctx().set_cursor_icon(CursorIcon::Grab);
                }
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

fn disabled_radio<T: PartialEq<T> + std::fmt::Display>(ui: &mut Ui, value: &T, color: Color32) {
    ui.radio(false, colored_text(&format!("{value}"), color))
        .on_hover_cursor(CursorIcon::NotAllowed);
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

pub fn sleek_button(ui: &mut Ui, string: &str, on_click: impl FnOnce()) {
    Frame {
        outer_margin: bevy_egui::egui::epaint::Margin::same(1.),
        inner_margin: bevy_egui::egui::epaint::Margin::same(5.),
        stroke: bevy_egui::egui::epaint::Stroke {
            width: 1.0,
            color: Color32::GRAY,
        },
        shadow: bevy_egui::egui::epaint::Shadow::NONE,
        ..default()
    }
    .show(ui, |ui| {
        if ui.label(text(string)).on_hover_cursor(CursorIcon::PointingHand).clicked() {
            on_click();
        }
    });
}
