use crate::{elements::PrincipalDirection, ActionEvent, Configuration, DrawLoopType, RenderType, SolutionResource};
use bevy::prelude::*;
use bevy_egui::egui::{emath::Numeric, Color32, RichText, Slider, TopBottomPanel, Ui};

pub fn ui(
    mut egui_ctx: bevy_egui::EguiContexts,
    mut ev_w: EventWriter<ActionEvent>,
    mut conf: ResMut<Configuration>,
    mut mesh_resmut: ResMut<crate::MeshResource>,
    mut solution: ResMut<SolutionResource>,
) {
    TopBottomPanel::top("panel").show(egui_ctx.ctx_mut(), |ui| {
        sep(ui);
        ui.horizontal(|ui| {
            if ui.button("Load file").clicked() {
                if let Some(path) = rfd::FileDialog::new().add_filter("triangulated geometry", &["obj", "stl", "save"]).pick_file() {
                    ev_w.send(ActionEvent::LoadFile(path));
                }
            }

            if conf.source.is_empty() {
                ui.label("No file yet loaded.");
            } else {
                ui.label(format!(
                    "\tLoaded: {}\t\t(v: {}, e: {}, f: {})",
                    conf.source, conf.nr_of_vertices, conf.nr_of_edges, conf.nr_of_faces
                ));
            }
        });

        ui.add_space(15.);

        ui.horizontal(|ui| {
            ui.vertical(|ui| {
                ui.checkbox(&mut conf.interactive, "interactive");
                ui.checkbox(&mut conf.region_selection, "region selection");
                ui.checkbox(&mut conf.zone_selection, "zone selection");
                ui.checkbox(&mut conf.black, "black toggle");
            });

            ui.vertical(|ui| {
                ui.checkbox(&mut conf.draw_wireframe, "graph");
                ui.checkbox(&mut conf.draw_vertices, "vertices");
                ui.checkbox(&mut conf.draw_normals, "normals");
            });

            ui.vertical(|ui| {
                slider(ui, "alpha", &mut conf.alpha, 1..=20);
                slider(ui, "beta", &mut conf.beta, 1..=20);
            });

            ui.add_space(15.);

            ui.vertical(|ui| {
                for draw_loop_type in [DrawLoopType::None, DrawLoopType::Directed, DrawLoopType::Undirected] {
                    radio(ui, &mut conf.draw_loop_type, draw_loop_type);
                }
            });

            ui.vertical(|ui| {
                for direction in [PrincipalDirection::X, PrincipalDirection::Y, PrincipalDirection::Z] {
                    radio(ui, &mut conf.direction, direction);
                }
            });

            ui.add_space(15.);

            ui.vertical(|ui| {
                for render_type in [RenderType::Original, RenderType::Polycube] {
                    if radio(ui, &mut conf.render_type, render_type) {
                        mesh_resmut.as_mut();
                    }
                }
            });

            ui.vertical(|ui| {
                ui.label(format!("Selected: {:?}", conf.cur_selected));

                ui.label("Current solution:");
                if let Err(err) = &solution.primal {
                    warning(ui, &format!("ERROR: {err:?}"));
                } else {
                    okido(ui, "OK");
                }

                if let Some(selected_edge) = conf.cur_selected {
                    if let Some(sol) = solution.next.get(&selected_edge) {
                        ui.label("Selected candidate solution:");
                        if let Some((_, Err(err))) = sol {
                            warning(ui, &format!("ERROR: {err:?}"));
                        } else {
                            okido(ui, "OK");
                        }
                    }
                }
            });

            ui.add_space(15.);

            ui.vertical(|ui| {
                ui.label("Sublabels");
                for i in 0..3 {
                    if stepper(
                        ui,
                        ["X-loops", "Y-loops", "Z-loops"][i],
                        &mut conf.sides_mask[i],
                        0,
                        2_u32.pow(u32::try_from(solution.dual.side_ccs[i].len()).unwrap()) - 1,
                    ) {
                        let res = solution.dual.zone_graph(conf.sides_mask);
                        if let Err(err) = res {
                            solution.primal = Err(err);
                        }
                        solution.primal = Ok(solution.dual.primal());
                        mesh_resmut.as_mut();
                    }
                }
            });

            if ui.button("Export").clicked() {
                ev_w.send(ActionEvent::ExportState);
            };
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
