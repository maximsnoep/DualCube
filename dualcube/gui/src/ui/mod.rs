//! The egui user interface: the top menu bar, the dockable render viewports,
//! and the footer.
//!
//! Each panel is its own system (in its own module), run in order via
//! [`UiPlugin`]; `theme` and `widgets` hold the shared styling and widgets.

pub mod dock;
mod footer;
mod menu;
mod theme;
mod widgets;

use bevy::prelude::*;
use bevy_egui::egui::FontFamily::Proportional;
use bevy_egui::egui::{CornerRadius, FontFamily, FontId, TextStyle};
use bevy_egui::{EguiPrimaryContextPass, PrimaryEguiContext};
use theme::BG_COLOR;

/// Registers the UI resources and the panel systems.
pub struct UiPlugin;

impl Plugin for UiPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<dock::UiResource>()
            .add_plugins(bevy_egui::EguiPlugin::default())
            .add_observer(setup)
            .add_systems(
                EguiPrimaryContextPass,
                (menu::show, dock::show, footer::show).chain(),
            );
    }
}

/// Sets the fonts and the theme once the primary egui context is created.
fn setup(_: On<Add, PrimaryEguiContext>, mut ui: bevy_egui::EguiContexts) -> Result<(), BevyError> {
    println!("running ui setup");
    // Font
    let mut fonts = bevy_egui::egui::FontDefinitions::default();
    fonts.font_data.insert(
        String::from("font"),
        bevy_egui::egui::FontData::from_static(include_bytes!("../../assets/font.ttf")).into(),
    );
    fonts.font_data.insert(
        String::from("font2"),
        bevy_egui::egui::FontData::from_static(include_bytes!("../../assets/font2.ttf")).into(),
    );

    fonts.families.insert(
        FontFamily::Monospace,
        vec![
            String::from("font"),
            String::from("font2"), // fallback for some ascii symbols like →
        ],
    );
    fonts.families.insert(
        FontFamily::Proportional,
        vec![
            String::from("font"),
            String::from("font2"), // fallback
        ],
    );

    ui.ctx_mut()?.set_fonts(fonts);

    // Theme
    ui.ctx_mut()?.style_mut(|style| {
        let zero = CornerRadius::same(0);
        let mut visuals = bevy_egui::egui::Visuals::dark();

        #[cfg(feature = "light_mode")]
        {
            use bevy_egui::egui::epaint::AlphaFromCoverage;
            visuals.dark_mode = false;
            visuals.text_alpha_from_coverage = AlphaFromCoverage::DARK_MODE_DEFAULT;
            visuals.widgets = bevy_egui::egui::style::Widgets::light();
        }

        visuals.window_fill = BG_COLOR;
        visuals.panel_fill = BG_COLOR;

        visuals.widgets.open.corner_radius = zero;
        visuals.menu_corner_radius = zero;
        visuals.window_corner_radius = zero;
        visuals.widgets.noninteractive.corner_radius = zero;
        visuals.widgets.hovered.corner_radius = zero;
        visuals.widgets.active.corner_radius = zero;

        visuals.clip_rect_margin = 0.;

        style.visuals = visuals;

        style.text_styles = [
            (TextStyle::Heading, FontId::new(30.0, Proportional)),
            (TextStyle::Body, FontId::new(12.0, Proportional)),
            (TextStyle::Monospace, FontId::new(12.0, Proportional)),
            (TextStyle::Button, FontId::new(12.0, Proportional)),
            (TextStyle::Small, FontId::new(10.0, Proportional)),
        ]
        .into();

        style.interaction.selectable_labels = false;
    });

    Ok(())
}
