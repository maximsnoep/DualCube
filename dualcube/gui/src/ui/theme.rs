use crate::colors;
use bevy_egui::egui::{text, Color32, FontId, TextFormat};

pub const RED: Color32 = Color32::from_rgb(
    (colors::SNOEP_RED[0] * 255.) as u8,
    (colors::SNOEP_RED[1] * 255.) as u8,
    (colors::SNOEP_RED[2] * 255.) as u8,
);
pub const LIGHT_RED: Color32 = Color32::from_rgb(
    (colors::RED_LIGHT[0] * 255.) as u8,
    (colors::RED_LIGHT[1] * 255.) as u8,
    (colors::RED_LIGHT[2] * 255.) as u8,
);
pub const BLUE: Color32 = Color32::from_rgb(
    (colors::SNOEP_BLUE[0] * 255.) as u8,
    (colors::SNOEP_BLUE[1] * 255.) as u8,
    (colors::SNOEP_BLUE[2] * 255.) as u8,
);

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
pub fn to_color32(color: colors::Color) -> Color32 {
    Color32::from_rgb(
        (color[0] * 255.) as u8,
        (color[1] * 255.) as u8,
        (color[2] * 255.) as u8,
    )
}

/// The monospace text format used throughout the UI.
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
