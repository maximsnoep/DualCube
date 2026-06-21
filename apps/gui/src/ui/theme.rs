use crate::colors;
use bevy::prelude::*;
use bevy_egui::egui::FontFamily;
use bevy_egui::egui::FontFamily::Proportional;
use bevy_egui::egui::{Color32, CornerRadius, FontId, TextFormat, TextStyle, text};

pub const OK_GREEN: Color32 = Color32::from_rgb(
    (colors::OK_GREEN[0] * 255.) as u8,
    (colors::OK_GREEN[1] * 255.) as u8,
    (colors::OK_GREEN[2] * 255.) as u8,
);

pub const WARN_RED: Color32 = Color32::from_rgb(
    (colors::WARN_RED[0] * 255.) as u8,
    (colors::WARN_RED[1] * 255.) as u8,
    (colors::WARN_RED[2] * 255.) as u8,
);

pub const REGULAR_TEXT_SIZE: f32 = 12.;
pub const SMALL_TEXT_SIZE: f32 = 10.;

#[cfg(not(feature = "light_mode"))]
pub const TEXT_COLOR: Color32 = Color32::from_gray(255);
#[cfg(feature = "light_mode")]
pub const TEXT_COLOR: Color32 = Color32::from_gray(27);

#[cfg(not(feature = "light_mode"))]
pub const TEXT_COLOR2: Color32 = Color32::from_gray(160);
#[cfg(feature = "light_mode")]
pub const TEXT_COLOR2: Color32 = Color32::from_gray(90);

#[cfg(not(feature = "light_mode"))]
pub const BG_COLOR: Color32 = Color32::from_gray(27);
#[cfg(feature = "light_mode")]
pub const BG_COLOR: Color32 = Color32::from_gray(255);

#[cfg(not(feature = "light_mode"))]
pub const OUTLINE_COLOR: Color32 = Color32::from_gray(50);
#[cfg(feature = "light_mode")]
pub const OUTLINE_COLOR: Color32 = Color32::from_gray(200);

/// Converts one of our [`colors::Color`]s into an egui color.
pub fn to_color32(color: colors::Kolor) -> Color32 {
    Color32::from_rgb(
        (color[0] * 255.) as u8,
        (color[1] * 255.) as u8,
        (color[2] * 255.) as u8,
    )
}

/// Sets the theme once the primary egui context is created.
pub fn setup(ui: &mut bevy_egui::EguiContexts<'_, '_>) -> Result<(), BevyError> {
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
            (
                TextStyle::Body,
                FontId::new(REGULAR_TEXT_SIZE, Proportional),
            ),
            (
                TextStyle::Monospace,
                FontId::new(REGULAR_TEXT_SIZE, Proportional),
            ),
            (
                TextStyle::Button,
                FontId::new(REGULAR_TEXT_SIZE, Proportional),
            ),
            (TextStyle::Small, FontId::new(SMALL_TEXT_SIZE, Proportional)),
        ]
        .into();

        style.interaction.selectable_labels = false;
    });

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
        Proportional,
        vec![
            String::from("font"),
            String::from("font2"), // fallback
        ],
    );

    ui.ctx_mut()?.set_fonts(fonts);

    Ok(())
}

/// The monospace text format used throughout the UI.
pub fn text_format(size: f32, color: Color32) -> TextFormat {
    TextFormat {
        font_id: FontId {
            size,
            family: FontFamily::Monospace,
        },
        color,
        ..Default::default()
    }
}

pub fn sized_text(string: &str, size: f32, color: Color32) -> text::LayoutJob {
    let mut job = text::LayoutJob::default();
    job.append(string, 0.0, text_format(size, color));
    job
}

pub fn colored_text(string: &str, color: Color32) -> text::LayoutJob {
    sized_text(string, 12., color)
}

pub fn text(string: &str) -> text::LayoutJob {
    colored_text(string, TEXT_COLOR)
}
