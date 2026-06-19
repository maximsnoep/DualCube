//! Render objects (meshes and gizmos per scene), their visibility settings,
//! and the systems that keep the spawned entities in sync with them.

use super::Objects;
use crate::colors::Kolor;
use crate::resources::Configuration;
use bevy::prelude::*;
use bevy_toon::ToonMaterial;
use mehsh::prelude::*;
use mehsh_bevy;
use std::collections::{HashMap, HashSet};

#[derive(Default, Debug, Clone)]
pub struct MeshProperties {
    pub source: String,
    pub scale: f64,
    pub translation: Vector3D,
}

/// Marks an entity as spawned by [`respawn_renders`] (despawned on respawn).
#[derive(Component)]
pub struct Rendered {
    pub object: Objects,
}

/// Marks the input mesh, the target for raycasting in the interactive modes.
#[derive(Component)]
pub struct MainMesh;

/// A single renderable feature of a [`RenderObject`].
#[derive(Clone)]
pub enum RenderAsset {
    Mesh(bevy::mesh::Mesh),
    Gizmo {
        asset: GizmoAsset,
        line_width: f32,
        depth_bias: f32,
    },
}

/// All renderable features of one of the [`Objects`], keyed by label.
#[derive(Clone, Default)]
pub struct RenderObject {
    pub labels: Vec<String>,
    pub features: HashMap<String, RenderAsset>,
}

impl RenderObject {
    pub fn add(&mut self, label: &str, asset: RenderAsset) -> &mut Self {
        self.labels.push(label.to_owned());
        self.features.insert(label.to_owned(), asset);
        self
    }

    pub fn mesh<M: Tag>(
        &mut self,
        mesh: &mehsh::prelude::Mesh<M>,
        color_map: &HashMap<FaceKey<M>, Kolor>,
        label: &str,
    ) -> &mut Self {
        self.add(
            label,
            RenderAsset::Mesh(mehsh_bevy::to_bevy(mesh, color_map).0),
        )
    }

    pub fn gizmo(&mut self, gizmo: GizmoAsset, width: f32, depth: f32, label: &str) -> &mut Self {
        self.add(
            label,
            RenderAsset::Gizmo {
                asset: gizmo,
                line_width: width,
                depth_bias: depth,
            },
        )
    }
}

#[derive(Default, Resource)]
pub struct RenderObjectStore {
    pub objects: HashMap<Objects, RenderObject>,
}

impl RenderObjectStore {
    pub fn add_object(&mut self, object: Objects, render_object: RenderObject) {
        self.objects.insert(object, render_object);
    }
}

#[derive(Clone)]
pub struct RenderFeatureSetting {
    pub label: String,
    pub visible: bool,
}

impl PartialEq for RenderFeatureSetting {
    fn eq(&self, other: &Self) -> bool {
        self.label == other.label && self.visible == other.visible
    }
}

#[derive(Clone, Default, PartialEq)]
pub struct RenderObjectSetting {
    pub labels: Vec<String>,
    pub settings: HashMap<String, RenderFeatureSetting>,
}

#[derive(Default, Resource)]
pub struct RenderObjectSettingStore {
    pub objects: HashMap<Objects, RenderObjectSetting>,
    last_applied_objects: HashMap<Objects, RenderObjectSetting>,
}

/// Syncs the settings store with the object store: every feature gets a
/// visibility toggle, newly seen features start with their default visibility.
pub fn update_render_settings(
    render_object_store: Res<RenderObjectStore>,
    mut render_settings_store: ResMut<RenderObjectSettingStore>,
) {
    let default = |object: &Objects, label: &str| {
        matches!(
            (object, label),
            (Objects::InputMesh, "gray")
                | (Objects::InputMesh, "wireframe")
                | (Objects::Polycube, "gray")
                | (Objects::Polycube, "paths")
                | (Objects::Polycube, "flat paths")
                | (Objects::PolycubeMap, "colored")
                | (Objects::PolycubeMap, "triangles")
                | (Objects::PolycubeMap, "paths")
                | (Objects::PolycubeMap, "flat paths")
                | (Objects::QuadMesh, "gray")
                | (Objects::QuadMesh, "wireframe")
        )
    };

    if render_object_store.is_changed() {
        for (object, render_object) in &render_object_store.objects {
            let labels = render_object.labels.clone();
            let mut settings = render_settings_store
                .objects
                .get(object)
                .map_or_else(HashMap::new, |s| s.settings.clone());
            for feature_label in render_object.features.keys() {
                settings
                    .entry(feature_label.clone())
                    .or_insert(RenderFeatureSetting {
                        label: feature_label.clone(),
                        visible: default(object, feature_label),
                    });
            }

            let new_setting = RenderObjectSetting { labels, settings };
            if render_settings_store.objects.get(object) != Some(&new_setting) {
                render_settings_store.objects.insert(*object, new_setting);
            }
        }
    }
}

/// Despawns and respawns rendered entities for objects whose settings changed.
pub fn respawn_renders(
    mut commands: Commands,
    mut meshes: ResMut<Assets<bevy::mesh::Mesh>>,
    mut gizmos: ResMut<Assets<GizmoAsset>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut custom_materials: ResMut<Assets<ToonMaterial>>,
    configuration: Res<Configuration>,
    render_object_store: Res<RenderObjectStore>,
    mut render_settings_store: ResMut<RenderObjectSettingStore>,
    rendered_mesh_query: Query<(Entity, &Rendered)>,
) {
    let mut changed_objects = HashSet::new();
    if render_object_store.is_changed() {
        changed_objects.extend(render_object_store.objects.keys().copied());
    }
    for (&object, settings) in &render_settings_store.objects {
        if render_settings_store.last_applied_objects.get(&object) != Some(settings) {
            changed_objects.insert(object);
        }
    }
    for object in render_settings_store.last_applied_objects.keys() {
        if !render_settings_store.objects.contains_key(object) {
            changed_objects.insert(*object);
        }
    }

    if changed_objects.is_empty() {
        return;
    }
    info!(
        ?changed_objects,
        "Render settings changed; respawning render objects"
    );

    for (entity, rendered) in rendered_mesh_query.iter() {
        if changed_objects.contains(&rendered.object) {
            commands.entity(entity).despawn();
        }
    }

    let flat_material = materials.add(StandardMaterial {
        unlit: true,
        ..default()
    });
    let toon_material = custom_materials.add(ToonMaterial {
        view_dir: Vec3::new(0.0, 0.0, 1.0),
    });
    let background_material = materials.add(StandardMaterial {
        base_color: super::clear_color(&configuration),
        unlit: true,
        ..default()
    });

    // Go through changed render objects and spawn their visible features.
    for (&object, render_object) in &render_object_store.objects {
        if !changed_objects.contains(&object) {
            continue;
        }
        let Some(settings) = render_settings_store
            .objects
            .get(&object)
            .map(|s| &s.settings)
        else {
            continue;
        };
        let translation = Vec3::from(object);

        for (label, asset) in &render_object.features {
            if !settings.get(label).unwrap().visible {
                continue;
            }
            match asset {
                RenderAsset::Mesh(mesh) => {
                    let mut entity = commands.spawn((
                        Mesh3d(meshes.add(mesh.clone())),
                        Transform::from_translation(translation),
                        Rendered { object },
                    ));
                    // The polycube-like objects are unlit; the surface meshes
                    // are toon-shaded, except for the pre-shaded input mesh.
                    let unlit = matches!(object, Objects::Polycube | Objects::PolycubeMap)
                        || (object == Objects::InputMesh && label == "shaded");
                    if unlit {
                        entity.insert(MeshMaterial3d(flat_material.clone()));
                    } else {
                        entity.insert(MeshMaterial3d(toon_material.clone()));
                    }
                    if object == Objects::InputMesh {
                        entity.insert(MainMesh);
                    }
                }
                RenderAsset::Gizmo {
                    asset,
                    line_width,
                    depth_bias,
                } => {
                    commands.spawn((
                        Gizmo {
                            handle: gizmos.add(asset.clone()),
                            line_config: GizmoLineConfig {
                                width: *line_width,
                                joints: GizmoLineJoint::Round(4),
                                ..Default::default()
                            },
                            depth_bias: *depth_bias,
                        },
                        Transform::from_translation(translation),
                        Rendered { object },
                    ));
                }
            }
        }

        // Spawn a cover such that the object is view-blocked from the others.
        commands.spawn((
            Mesh3d(meshes.add(Sphere::new(400.))),
            MeshMaterial3d(background_material.clone()),
            Transform::from_translation(translation),
            Rendered { object },
        ));
    }

    render_settings_store.last_applied_objects = render_settings_store.objects.clone();
}
