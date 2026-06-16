//! The egui user interface: the top menu bar, the dockable render viewports,
//! and the footer.
//!
//! Each panel is its own system (in its own module), run in order via
//! [`UiPlugin`]; `theme` and `widgets` hold the shared styling and widgets.

pub mod dock;
mod font;
mod footer;
mod menu;
mod theme;
mod widgets;

use bevy::prelude::*;
use bevy_egui::{EguiPrimaryContextPass, PrimaryEguiContext};

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

/// Sets up egui once the primary context is created.
fn setup(_: On<Add, PrimaryEguiContext>, mut ui: bevy_egui::EguiContexts) -> Result<(), BevyError> {
    font::setup(&mut ui)?;
    theme::setup(&mut ui)?;
    Ok(())
}
