# Homology-preserving connectivity surgery

Design note for the legality criterion in `connectivity_surgery.rs`.

## Context

### What surgery does

Connectivity surgery starts from the contracted mesh — a closed orientable
2-manifold whose vertices have been pulled toward the medial axis by
Laplacian shrinking — and greedily collapses edges until the result is a
1D curve skeleton. The legality of each collapse decides what gets
preserved.

### What the legality check has been

1. **Strict Dey–Edelsbrunner link condition** (original): for every shared
   neighbour `w` of `u` and `v`, the face `(u, v, w)` must exist. This is
   exactly the condition for the collapse to be a homotopy equivalence on
   a closed 2-manifold. Empirically it gets stuck very early on some
   inputs (an old report: "all genus 0 meshes end up at 2 patches").

2. **Homology-preservation** (recent): probe what β₁ of the simplicial
   complex would become after a hypothetical collapse and reject if it
   would change. This admits collapses the strict link condition refuses,
   but allows the complex to become non-manifold. Non-manifold spikes
   then inflate β₁ with spurious 1-skeleton cycles that block all later
   collapses.

3. **Homology + manifold** (current): both checks must pass. Genus 0
   meshes (cylinder test) collapse fully to V=2, E=1, F=0. Genus 1 meshes
   (bob.stl) get stuck on a clean closed sub-complex very near the
   theoretical minimum triangulation (V=9, E=27, F=18 for bob; the
   Möbius–Kantor torus has V=7, E=21, F=14).

### Why genus > 0 inevitably gets stuck under (3)

For a closed orientable surface of genus *g*, every link-condition-
preserving edge collapse is a homotopy equivalence. So β₁ stays at *2g*
throughout, and the algorithm cannot reduce below the minimum
triangulation of a genus-*g* surface (14 faces for *g*=1, etc.). This is
a topological floor, not an algorithmic deficiency.

But the actual end goal is a 1D skeleton **homotopic to the input solid**
— the handlebody *V* bounded by the surface *S*. *V* has β₁ = *g* (not
*2g*); *g* of the surface's cycles ("handle loops") bound disks in *V*
and should disappear in the skeleton, while the other *g* ("tunnel
loops") need to survive as cycles. So we want an algorithm that admits
collapses that reduce β₁ from *2g* to *g* — but only the right ones.

## Proposed solution

### Core idea

The handle subspace
`K := ker(H₁(S; ℤ/2) → H₁(V; ℤ/2)) ⊆ H₁(S; ℤ/2)`
is canonical and has dimension *g*. Tunnel loops are *not* canonical (a
choice of complement to *K*), so phrase the criterion in terms of *K*.

We precompute a basis `c₁, ..., c_g` for *K* on the input surface, then
track those *g* chains alongside ∂₂ throughout surgery and require:

> **Legality criterion (★)**:
> ```
> β₁(X^{(t)}) − dim K^{(t)} = g
> ```
> where `K^{(t)}` is the subspace of `H₁(X^{(t)}; ℤ/2)` spanned by the
> chain-image evolutions of `c₁, ..., c_g`.

### Precomputation (one-time, on the input mesh, before contraction)

1. Tetrahedralize the closed input surface *S* to obtain a tet complex *T*
   of the solid *V* (e.g., via TetGen FFI).
2. Compute `∂₂^T : C₂(T; ℤ/2) → C₁(T; ℤ/2)`.
3. Find *K* by linear algebra over ℤ/2: a surface 1-cycle `c ∈ Z₁(S)` is
   in *K* iff it is a boundary in *T*, i.e.
   ```
   K = Z₁(S) ∩ im(∂₂^T)
   ```
   Solve via sparse F2 RREF (the existing `f2_rref.rs` module).
4. Extract a basis `c₁, ..., c_g ∈ C₁(S; ℤ/2)`. Store each as a sorted
   list of edge column indices (same representation as a ∂₂ row).

### Surgery (modified)

Maintain alongside ∂₂ during surgery the *g* chains `c₁^{(t)}, ..., c_g^{(t)}`
representing the handle subspace at step *t*. For each candidate collapse
`u → v`:

1. Compute the existing `CollapseDelta`.
2. Apply the *same column-merge operations* to each `cᵢ` that we apply to
   ∂₂'s rows. Edge collapse is a chain map, and on 1-chains that chain
   map is exactly the column-merge — independent of whether the chain is
   a ∂₂ row or one of the `cᵢ`. So the snapshot of each `cᵢ'` comes for
   free from the same operation we already use for ∂₂.
3. Compute `rank(∂₂')`.
4. Compute `dim K^{(t+1)}` by augmenting `∂₂'` with the rows
   `c₁', ..., c_g'`, taking that matrix's rank, and subtracting
   `rank(∂₂')`.
5. Accept iff (★): `β₁(X') − dim K' = g`, where
   `β₁(X') = E' − V' + 1 − rank(∂₂')`.

### Drop the manifold link condition

The strict link condition is **incompatible** with this method. Every
link-preserving collapse is a homotopy equivalence, which preserves H₁
entirely and never kills a handle. We rely on (★) alone to reject
spurious-cycle creation; the proof below shows it suffices.

## Proof of correctness

### Lemma (induction invariant)

If every accepted collapse satisfies (★), then at every step *t* the
natural map `ρ_t : H₁(X^{(t)}; ℤ/2) → H₁(V; ℤ/2)` is surjective with
kernel `⟨[c₁^{(t)}], ..., [c_g^{(t)}]⟩`.

**Base case** (*t* = 0): `ρ₀ = i_*` is surjective by handlebody theory,
with `ker(ρ₀) = K = ⟨[c₁], ..., [c_g]⟩` by construction.

**Inductive step**: an edge collapse *q* induces a chain map
`q_# : C_*(X^{(t)}) → C_*(X^{(t+1)})`. Chain maps preserve cycles and
boundaries, so descend to `q_* : H₁(X^{(t)}) → H₁(X^{(t+1)})`, with
`q_*([cᵢ^{(t)}]) = [cᵢ^{(t+1)}]` by definition of the chain-map action.

`q_*` may have a kernel — exactly the cycle class being killed by this
collapse. By criterion (★), the dimension of `ker(q_*)` equals the
corresponding drop in `dim K`, so the killed class lies in
`⟨[cᵢ^{(t)}]⟩`. Hence `ρ_t` factors uniquely through `q_*` to give a
well-defined `ρ_{t+1}` with kernel `⟨[cᵢ^{(t+1)}]⟩`. Surjectivity
transfers along the factorisation, and rank-nullity then reproduces (★)
on `X^{(t+1)}`.  ∎

### Terminal claim

If surgery terminates with `F^{(n)} = 0` (so `X^{(n)}` is a 1-complex)
and `cᵢ^{(n)} = 0` as 1-chains for all *i*, then
> ```
> X^{(n)} ≃ V
> ```
(homotopy equivalence).

**Proof**: `X^{(n)}` is a connected graph, so
`π₁(X^{(n)}) = F_{β₁(X^{(n)})}` (free of rank β₁). The invariant gives
`β₁(X^{(n)}) = g + dim K^{(n)} = g + 0 = g`. Hence
`π₁(X^{(n)}) = F_g = π₁(V)`. Both spaces are `K(F_g, 1)` (graphs are
aspherical; handlebodies are aspherical), so the iso of fundamental
groups is realised by a homotopy equivalence.  ∎

### Why this is the right invariant

Every candidate collapse falls into exactly one case, and (★) sorts them
correctly:

| Case | Δβ₁ | Δ(dim *K*) | Δ(β₁ − dim *K*) | Verdict |
|---|---|---|---|---|
| Topology-preserving | 0 | 0 | 0 | accept |
| Handle-killing | −1 | −1 | 0 | accept |
| Tunnel-killing | −1 | 0 | −1 | reject |
| Spurious 1-skeleton cycle | +1 | 0 | +1 | reject |

The criterion needs no manual classification of *which* case a candidate
falls into — the algebra forces it.

## What the method can't promise

### Termination

Termination at `F = 0` with `cᵢ = 0` is not automatic. The criterion
*admits* enough collapses to reach that state, but the greedy cost
function (shape QEM + sampling) must actually *choose* handle-killing
collapses when relevant — they have to look "cheap" to the cost. After
geometric contraction, handle loops are usually geometrically tiny
(contraction shrinks them to the medial axis), so in practice their
collapses are cheap, but this should be verified empirically.

If greedy doesn't naturally find handle kills, a small bias in the cost
function (e.g., a small negative term proportional to the candidate's
contribution to reducing `dim K`) would force them. Such a bias is
provably homotopy-safe because (★) still gates legality — bias only
affects ordering, not what's allowed.

### Cost and dependencies

- **Tetrahedralization**: TetGen via FFI is the practical choice (mature,
  robust, BSD license; pure-Rust alternatives are immature). For bob
  (~5k surface vertices) the tet mesh is on the order of 30k tets; the
  kernel computation is sparse F2 elimination on a ~30k × ~16k matrix,
  feasible with `f2_rref`.
- **Per-collapse legality check**: same snapshot + reduce as today, plus
  augmenting the snapshot with *g* extra rows and one more rank
  computation. Asymptotically the same complexity.
- **Storage**: *g* extra 1-chains, each sparse. Negligible.

### Robustness of tetrahedralization

The contracted mesh has self-intersections (it's been collapsed toward
the medial axis), so tetrahedralization must be done on the **original
input mesh**, not the contracted one. Handle cycles are computed on the
input mesh's vertex IDs and translated to contraction-mesh IDs via the
existing `vertex_to_original` map (used in reverse).
