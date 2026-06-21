//! The central dock area: one viewport tab per render object.

use super::theme::{BG_COLOR, OUTLINE_COLOR, TEXT_COLOR};
use crate::render::Objects;
use crate::render::camera::{CameraFor, CameraHandles};
use crate::render::store::{RenderObjectSetting, RenderObjectSettingStore};
use crate::resources::Configuration;
use bevy::prelude::*;
use bevy_egui::egui::{
    CentralPanel, Color32, CornerRadius, Frame, Id, Margin, Pos2, Sense, Stroke, Ui,
};
use egui_dock::tab_viewer::OnCloseResponse;
use egui_dock::{DockArea, DockState, NodeIndex, Style};
use std::collections::HashMap;

/// The dock tree: which viewport tab is shown where.
#[derive(Resource)]
pub struct UiResource {
    pub tree: DockState<Objects>,
}

impl Default for UiResource {
    fn default() -> Self {
        UiResource {
            tree: { DockState::new(vec![Objects::InputMesh, Objects::Polycube]) },
        }
    }
}

struct TabViewer {
    egui_handles: Vec<bevy_egui::egui::TextureId>,
    render_settings: HashMap<Objects, RenderObjectSetting>,
    axes_handle: bevy_egui::egui::TextureId,
}

impl egui_dock::TabViewer for TabViewer {
    type Tab = Objects;

    fn title(&mut self, tab: &mut Self::Tab) -> bevy_egui::egui::WidgetText {
        tab.to_string().into()
    }

    fn tab_style_override(
        &self,
        tab: &Self::Tab,
        _global_style: &egui_dock::TabStyle,
    ) -> Option<egui_dock::TabStyle> {
        let mut default_style = egui_dock::TabStyle::default();
        default_style.tab_body.stroke = Stroke::new(0., Color32::from_rgb(255, 0, 0));
        default_style.tab_body.inner_margin = Margin::same(0);

        match tab {
            Objects::InputMesh => {
                default_style.tab_body.bg_fill = Color32::TRANSPARENT;
            }
            _ => {
                default_style.tab_body.bg_fill = BG_COLOR;
            }
        }

        default_style.active.corner_radius = CornerRadius::same(0);
        default_style.active.bg_fill = BG_COLOR;
        default_style.active.outline_color = BG_COLOR;

        default_style.active_with_kb_focus = default_style.active.clone();
        default_style.focused = default_style.active.clone();
        default_style.focused_with_kb_focus = default_style.active.clone();
        default_style.hovered = default_style.active.clone();
        default_style.inactive = default_style.active.clone();
        default_style.inactive_with_kb_focus = default_style.active.clone();

        default_style.active.text_color = TEXT_COLOR;
        default_style.active_with_kb_focus.text_color = TEXT_COLOR;

        default_style.inactive.text_color = Color32::from_gray(100);
        default_style.inactive_with_kb_focus.text_color = Color32::from_gray(100);

        default_style.hovered.text_color = Color32::from_gray(150);
        default_style.focused.text_color = TEXT_COLOR;
        default_style.focused_with_kb_focus.text_color = TEXT_COLOR;

        Some(default_style)
    }

    fn allowed_in_windows(&self, _tab: &mut Self::Tab) -> bool {
        false
    }

    fn context_menu(
        &mut self,
        ui: &mut Ui,
        tab: &mut Self::Tab,
        _surface: egui_dock::SurfaceIndex,
        _node: NodeIndex,
    ) {
        if let Some(local_copy) = self.render_settings.get_mut(tab) {
            for label in &local_copy.labels {
                if let Some(setting) = local_copy.settings.get_mut(label) {
                    ui.checkbox(&mut setting.visible, label.to_owned());
                }
            }
        } else {
            ui.label("_______________________________");
        }
    }

    fn ui(&mut self, ui: &mut Ui, tab: &mut Self::Tab) {
        Frame {
            stroke: Stroke {
                width: 5.0,
                color: BG_COLOR,
            },
            ..default()
        }
        .show(ui, |ui| {
            Frame {
                stroke: Stroke {
                    width: 1.0,
                    color: OUTLINE_COLOR,
                },
                ..default()
            }
            .show(ui, |ui| {
                let response = match tab {
                    Objects::InputMesh => ui.allocate_exact_size(ui.available_size(), Sense::all()),

                    _ => {
                        let egui_handle = match tab {
                            Objects::PolycubeMap => self.egui_handles[0],
                            Objects::QuadMesh => self.egui_handles[1],
                            Objects::Polycube => self.egui_handles[2],
                            _ => unreachable!(),
                        };
                        let [w, h] = ui.available_size().into();

                        // Crop the square render texture to the viewport's
                        // aspect ratio.
                        let (min, max) = if w > h {
                            let offset = (1.0 - h / w) / 2.0;
                            (Pos2::new(0., offset), Pos2::new(1.0, 1.0 - offset))
                        } else {
                            let offset = (1.0 - w / h) / 2.0;
                            (Pos2::new(offset, 0.), Pos2::new(1.0 - offset, 1.0))
                        };

                        return ui.add(
                            bevy_egui::egui::widgets::Image::new(
                                bevy_egui::egui::load::SizedTexture::new(egui_handle, [w, h]),
                            )
                            .uv(bevy_egui::egui::Rect::from_min_max(min, max)),
                        );
                    }
                };

                ui.put(
                    bevy_egui::egui::Rect::from_two_pos(
                        response.0.left_bottom(),
                        Pos2::new(response.0.left() + 200., response.0.bottom() - 200.),
                    ),
                    bevy_egui::egui::widgets::Image::new(bevy_egui::egui::load::SizedTexture::new(
                        self.axes_handle,
                        [200., 200.],
                    )),
                )
            });
        });
    }

    fn closeable(&mut self, _tab: &mut Self::Tab) -> bool {
        false
    }

    fn scroll_bars(&self, _tab: &Self::Tab) -> [bool; 2] {
        [false, false]
    }

    fn id(&mut self, tab: &mut Self::Tab) -> Id {
        Id::new(self.title(tab).text())
    }

    fn on_tab_button(&mut self, _tab: &mut Self::Tab, _response: &bevy_egui::egui::Response) {}

    fn on_close(&mut self, _tab: &mut Self::Tab) -> OnCloseResponse {
        OnCloseResponse::Ignore
    }

    fn on_add(&mut self, _surface: egui_dock::SurfaceIndex, _node: NodeIndex) {}

    fn add_popup(&mut self, _ui: &mut Ui, _surface: egui_dock::SurfaceIndex, _node: NodeIndex) {}

    fn force_close(&mut self, _tab: &mut Self::Tab) -> bool {
        false
    }

    fn clear_background(&self, _tab: &Self::Tab) -> bool {
        true
    }
}

/// Shows the central dock area with the render viewports.
pub fn show(
    mut egui_ctx: bevy_egui::EguiContexts<'_, '_>,
    conf: Res<'_, Configuration>,
    image_handle: Res<'_, CameraHandles>,
    mut ui_resource: ResMut<'_, UiResource>,
    mut render_setting_store: ResMut<'_, RenderObjectSettingStore>,
    axes_texture: Res<'_, bevy_axes_gizmo::AxesGizmoTexture>,
) -> Result<(), BevyError> {
    let axes_handle =
        egui_ctx.add_image(bevy_egui::EguiTextureHandle::Strong(axes_texture.0.clone()));

    let mut egui_handles = vec![];
    for obj in conf.window_shows_object.iter() {
        let egui_handle = egui_ctx.add_image(bevy_egui::EguiTextureHandle::Strong(
            image_handle.map.get(&CameraFor(*obj)).unwrap().clone(),
        ));
        egui_handles.push(egui_handle);
    }

    CentralPanel::default()
        .frame(Frame {
            stroke: Stroke {
                width: 20.0,
                color: BG_COLOR,
            },
            fill: Color32::TRANSPARENT,
            ..default()
        })
        .show(egui_ctx.ctx_mut()?, |ui| {
            let dock_area = DockArea::new(&mut ui_resource.tree)
                .show_leaf_collapse_buttons(false)
                .show_leaf_close_all_buttons(false);
            let mut dock_area_style = Style::from_egui(ui.style());
            dock_area_style.dock_area_padding = Some(Margin {
                left: 10,
                right: 10,
                top: 5,
                bottom: 40,
            });
            dock_area_style.tab_bar.corner_radius = CornerRadius::same(0);
            dock_area_style.tab_bar.bg_fill = BG_COLOR;
            dock_area_style.tab_bar.hline_color = BG_COLOR;
            dock_area_style.separator.width = 1.;
            dock_area_style.separator.color_dragged = BG_COLOR;
            dock_area_style.separator.color_hovered = BG_COLOR;
            dock_area_style.separator.color_idle = BG_COLOR;

            dock_area_style.overlay.selection_color =
                Color32::from_rgba_unmultiplied(50, 50, 50, 100);
            dock_area_style.overlay.overlay_type = egui_dock::OverlayType::HighlightedAreas;

            let settings_copy = &render_setting_store.objects;

            let mut tab_viewer = TabViewer {
                egui_handles: egui_handles.clone(),
                render_settings: settings_copy.clone(),
                axes_handle,
            };
            dock_area
                .style(dock_area_style)
                .show(ui.ctx(), &mut tab_viewer);

            // Write back any visibility changes made in the context menus.
            if settings_copy != &tab_viewer.render_settings {
                render_setting_store.objects = tab_viewer.render_settings.clone();
            }
        });

    Ok(())
}
