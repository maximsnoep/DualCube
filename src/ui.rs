#![warn(clippy::all, clippy::pedantic, clippy::nursery, clippy::cargo)]

use crate::{dual::PrincipalDirection, ActionEvent, Configuration};
use bevy::prelude::*;
use bevy_egui::egui::{emath::Numeric, Slider, TopBottomPanel, Ui};

pub fn ui(
    mut egui_ctx: bevy_egui::EguiContexts,
    mut ev_w: EventWriter<ActionEvent>,
    mut conf: ResMut<Configuration>,
) {
    TopBottomPanel::top("panel").show(egui_ctx.ctx_mut(), |ui| {
        sep(ui);
        ui.horizontal(|ui| {
            if ui.button("Load file (supported: .stl)").clicked() {
                if let Some(path) = rfd::FileDialog::new()
                    .add_filter("triangulated geometry", &["stl", "poc", "obj"])
                    .pick_file()
                {
                    ev_w.send(ActionEvent::LoadFile(path));
                }
            }

            if !conf.source.is_empty() {
                ui.label(format!(
                    "\tLoaded: {}\t\t(v: {}, e: {}, f: {})",
                    conf.source, conf.nr_of_vertices, conf.nr_of_edges, conf.nr_of_faces
                ));
            } else {
                ui.label("No file yet loaded.");
            }
        });

        ui.add_space(10.);

        ui.horizontal(|ui| {
            ui.vertical(|ui| {
                ui.checkbox(&mut conf.draw_wireframe, "graph");
                ui.checkbox(&mut conf.draw_vertices, "vertices");
                ui.checkbox(&mut conf.draw_normals, "normals");
            });

            ui.add_space(10.);

            ui.add_space(10.);

            ui.vertical(|ui| {
                for direction in [
                    PrincipalDirection::X,
                    PrincipalDirection::Y,
                    PrincipalDirection::Z,
                ] {
                    radio(ui, &mut conf.direction, direction);
                }
            });

            ui.add_space(10.);

            ui.vertical(|ui| {
                slider(ui, "angle", &mut conf.angle_filter, 0.0..=180.0);
                slider(ui, "samples", &mut conf.samples, 1..=2000);
                button(ui, "add", &mut ev_w, ActionEvent::AddLoop(conf.direction));
            });

            ui.vertical(|ui| {
                ui.horizontal(|ui| {
                    if ui.button("<<").clicked() {
                        conf.id_selector -= 1;
                    };
                    ui.label(format!("  id {}  ", conf.id_selector));
                    if ui.button(">>").clicked() {
                        conf.id_selector += 1;
                    };
                });
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

fn button(ui: &mut Ui, label: &str, ev_writer: &mut EventWriter<ActionEvent>, ev: ActionEvent) {
    if ui.button(label).clicked() {
        ev_writer.send(ev);
    }
}

fn radio<T: PartialEq<T> + std::fmt::Debug>(ui: &mut Ui, item: &mut T, value: T) {
    if ui.radio(*item == value, format!("{value:?}")).clicked() {
        *item = value;
    }
}
