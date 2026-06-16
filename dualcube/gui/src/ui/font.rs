use bevy::prelude::*;
use bevy_egui::egui::FontFamily;

/// Sets the fonts once the primary egui context is created.
pub fn setup(ui: &mut bevy_egui::EguiContexts) -> Result<(), BevyError> {
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

    Ok(())
}
