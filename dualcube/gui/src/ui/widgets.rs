//! Small reusable egui widgets.

use super::theme::{colored_text, sized_text, text, RED, TEXT_COLOR, TEXT_COLOR2};
use bevy::prelude::Time;
use bevy_egui::egui::{emath, Color32, RichText, Slider, Ui};

pub fn sep(ui: &mut Ui) {
    ui.add_space(5.);
    ui.separator();
    ui.add_space(5.);
}

pub fn space(ui: &mut Ui) {
    ui.add_space(5.);
}

pub fn label(ui: &mut Ui, label: &str, size: f32, color: Color32) {
    ui.label(sized_text(label, size, color));
}

pub fn slider<T: emath::Numeric>(
    ui: &mut Ui,
    label: &str,
    value: &mut T,
    range: std::ops::RangeInclusive<T>,
    logarithmic: bool,
) {
    ui.add(
        Slider::new(value, range)
            .logarithmic(logarithmic)
            .text(text(label)),
    );
}

pub fn log_slider(ui: &mut Ui, label: &str, value: &mut f32, scale: f32) {
    slider(ui, label, value, 0.0..=scale, true);
}

pub fn radio<T: PartialEq<T> + std::fmt::Display>(
    ui: &mut Ui,
    item: &mut T,
    value: T,
    color: Color32,
) -> bool {
    if ui
        .radio(*item == value, colored_text(&format!("{value}"), color))
        .clicked()
    {
        *item = value;
        true
    } else {
        false
    }
}

pub fn menu_button(ui: &mut Ui, label: &str, f: impl FnOnce(&mut Ui)) {
    ui.menu_button(RichText::new(label).color(TEXT_COLOR).size(12.), f);
}

pub fn sleek_button(ui: &mut Ui, label: &str) -> bool {
    click_button(ui, label, TEXT_COLOR)
}

pub fn sleek_button_warn(ui: &mut Ui, label: &str) -> bool {
    click_button(ui, label, RED)
}

pub fn sleek_button_unfocused(ui: &mut Ui, label: &str) -> bool {
    click_button(ui, label, TEXT_COLOR2)
}

fn click_button(ui: &mut Ui, label: &str, color: Color32) -> bool {
    ui.button(RichText::new(label).color(color).size(12.))
        .clicked()
}

/// A small looping animation indicating that a job is running.
pub fn timer_animation(time: &Time) -> String {
    let frequency = 6.0;
    let animation = ["●○○○", "○●○○", "○○●○", "○○○●", "○○●○", "○●○○"];
    let index = (time.elapsed_secs() * frequency) as usize % animation.len();
    animation[index].to_string()
}
