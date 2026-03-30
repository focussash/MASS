# MASS Muscle Mapping (284 Muscles)

> Classification of all 284 muscles in MASS's `muscle284.xml` for the
> Exoskeleton RL project. Identifies paper's 208-muscle and 108 lower-limb subsets.

---

## Summary

| Category | Count | Description |
|----------|-------|-------------|
| Total muscles | 284 | 142 unique x 2 sides (L/R) |
| Lower-limb | 178 | Touches Pelvis/Femur/Tibia/Talus/Foot |
| Trunk-arm | 26 | Connects trunk (Spine/Torso) to arm/shoulder |
| Trunk-only | 2 | Only touches Spine/Torso |
| Arm/hand | 62 | Pure arm/hand/shoulder (no trunk) |
| Neck/head | 16 | Pure neck/head muscles |

### Paper Subsets (Estimated)

| Paper Requirement | Estimated Count | Rule |
|-------------------|----------------|------|
| **208 used muscles** | ~206 | Exclude pure arm/hand (62) + neck/head (16) = 78 excluded |
| **108 lower-limb (exo reward)** | ~108 | Hip-crossing (92) + selected knee/ankle (~16) |

The exact 208 and 108 subsets are not specified in the paper. The estimates above
are based on anatomical reasoning. The 2-muscle discrepancy (206 vs 208) could mean
the paper includes 2 additional trunk-arm muscles. This can be tuned during Phase 4.

---

## Hip-Crossing Muscles (92 total)

These muscles cross the hip joint (attach to both Pelvis/Spine/Torso AND Femur).
They are the primary targets for the exo reward's muscle minimization term.

### Hip Flexors (cross anterior hip)
| Idx | Name | f0 (N) | Bodies |
|-----|------|--------|--------|
| 164 | L_Psoas_Major | 1000.0 | Pelvis, FemurL, Spine |
| 165 | R_Psoas_Major | 1000.0 | Pelvis, FemurR, Spine |
| 166 | L_Psoas_Major1 | 1000.0 | FemurL, Spine |
| 167 | R_Psoas_Major1 | 1000.0 | FemurR, Spine |
| 168 | L_Psoas_Major2 | 1000.0 | FemurL, Spine |
| 169 | R_Psoas_Major2 | 1000.0 | FemurR, Spine |
| 170 | L_Psoas_Minor | 1000.0 | Pelvis, Spine |
| 171 | R_Psoas_Minor | 1000.0 | Pelvis, Spine |
| 260 | L_iliacus | 1000.0 | FemurL, Pelvis |
| 261 | R_iliacus | 1000.0 | FemurR, Pelvis |
| 262 | L_iliacus1 | 1000.0 | FemurL, Pelvis |
| 263 | R_iliacus1 | 1000.0 | FemurR, Pelvis |
| 264 | L_iliacus2 | 1000.0 | FemurL, Pelvis |
| 265 | R_iliacus2 | 1000.0 | FemurR, Pelvis |
| 176 | L_Rectus_Femoris | 537.6 | FemurL, Pelvis, TibiaL |
| 177 | R_Rectus_Femoris | 537.6 | FemurR, Pelvis, TibiaR |
| 178 | L_Rectus_Femoris1 | 537.6 | FemurL, Pelvis, TibiaL |
| 179 | R_Rectus_Femoris1 | 537.6 | FemurR, Pelvis, TibiaR |
| 192 | L_Sartorius | 1000.0 | FemurL, Pelvis, TibiaL |
| 193 | R_Sartorius | 1000.0 | FemurR, Pelvis, TibiaR |
| 232 | L_Tensor_Fascia_Lata | 1000.0 | FemurL, Pelvis |
| 233 | R_Tensor_Fascia_Lata | 1000.0 | FemurR, Pelvis |
| 234 | L_Tensor_Fascia_Lata1 | 1000.0 | FemurL, Pelvis, TibiaL |
| 235 | R_Tensor_Fascia_Lata1 | 1000.0 | FemurR, Pelvis, TibiaR |
| 236 | L_Tensor_Fascia_Lata2 | 1000.0 | FemurL, Pelvis, TibiaL |
| 237 | R_Tensor_Fascia_Lata2 | 1000.0 | FemurR, Pelvis, TibiaR |
| 156 | L_Pectineus | 1000.0 | FemurL, Pelvis |
| 157 | R_Pectineus | 1000.0 | FemurR, Pelvis |

### Hip Extensors (cross posterior hip)
| Idx | Name | f0 (N) | Bodies |
|-----|------|--------|--------|
| 96 | L_Gluteus_Maximus | 382.0 | FemurL, Pelvis |
| 97 | R_Gluteus_Maximus | 382.0 | FemurR, Pelvis |
| 98-105 | Gluteus_Maximus 1-4 (L/R) | 382.0 | Pelvis, Femur |
| 26 | L_Bicep_Femoris_Longus | 600.0 | FemurL, Pelvis, TibiaL |
| 27 | R_Bicep_Femoris_Longus | 600.0 | FemurR, Pelvis, TibiaR |
| 198 | L_Semimembranosus | 1000.0 | FemurL, Pelvis, TibiaL |
| 199 | R_Semimembranosus | 1000.0 | FemurR, Pelvis, TibiaR |
| 200 | L_Semimembranosus1 | 1000.0 | FemurL, Pelvis, TibiaL |
| 201 | R_Semimembranosus1 | 1000.0 | FemurR, Pelvis, TibiaR |
| 204 | L_Semitendinosus | 1000.0 | FemurL, Pelvis, TibiaL |
| 205 | R_Semitendinosus | 1000.0 | FemurR, Pelvis, TibiaR |

### Hip Abductors/Adductors/Rotators
| Idx | Name | f0 (N) | Bodies |
|-----|------|--------|--------|
| 2-19 | Adductor Brevis/Longus/Magnus (L/R, multiple heads) | 151-259 N | Pelvis, Femur |
| 106-117 | Gluteus Medius/Minimus (L/R, multiple heads) | 1000.0 | Pelvis, Femur |
| 118 | L_Gracilis | 1000.0 | FemurL, Pelvis, TibiaL |
| 119 | R_Gracilis | 1000.0 | FemurR, Pelvis, TibiaR |
| 120-121 | Inferior_Gemellus (L/R) | 1000.0 | Pelvis, Femur |
| 140-141 | Obturator_Externus (L/R) | 1000.0 | Pelvis, Femur |
| 142-143 | Obturator_Internus (L/R) | 1000.0 | Pelvis, Femur |
| 160-163 | Piriformis (L/R, 2 heads) | 1000.0 | Pelvis, Femur |
| 172-173 | Quadratus_Femoris (L/R) | 1000.0 | Pelvis, Femur |
| 228-229 | Superior_Gemellus (L/R) | 1000.0 | Pelvis, Femur |

### Total hip-crossing: 92 muscles (46 per side)

---

## Non-Hip Lower-Limb Muscles (86 muscles)

These touch lower-limb bodies but do NOT cross the hip joint.

### Knee Muscles (Femur to Tibia)
| Muscle Group | Count | Bodies |
|-------------|-------|--------|
| Bicep_Femoris_Short (2 heads, L/R) | 4 | Femur, Tibia |
| Vastus Intermedius/Lateralis/Medialis (multiple heads, L/R) | 16 | Femur, Tibia |
| Popliteus (L/R) | 2 | Femur, Tibia |
| Plantaris (L/R) | 2 | Femur, Tibia, Talus |

### Ankle/Foot Muscles (Tibia to Foot)
| Muscle Group | Count | Bodies |
|-------------|-------|--------|
| Gastrocnemius Lateral/Medial (L/R) | 4 | Femur, Tibia, Talus |
| Soleus (2 heads, L/R) | 4 | Tibia, Talus |
| Tibialis Anterior/Posterior (L/R) | 4 | Tibia, Talus/Foot |
| Peroneus Brevis/Longus/Tertius (L/R) | 8 | Tibia, Talus/Foot |
| Extensor Digitorum/Hallucis Longus (L/R) | 10 | Tibia, Foot |
| Flexor Digitorum/Hallucis Longus (L/R) | 12 | Tibia, Foot |
| Flexor Digiti Minimi Brevis Foot (L/R) | 2 | Talus, Foot |

### Pelvis-only (touch Pelvis but not Femur)
| Muscle Group | Count | Bodies |
|-------------|-------|--------|
| Psoas_Minor (L/R) | 2 | Pelvis, Spine |
| Quadratus_Lumborum (L/R) | 2 | Pelvis, Torso |

---

## Excluded Muscles (Paper's ~76 exclusions)

### Pure Arm/Hand (62 muscles)
Muscles entirely within the arm/hand with no trunk attachment:
- Abductor_Pollicis_Longus, Anconeous, Bicep_Brachii (Long/Short), Brachialis,
  Brachioradialis, Extensor Carpi/Digiti/Pollicis, Flexor Carpi/Digitorum/Pollicis,
  Palmaris_Longus, Triceps (Lateral/Long/Medial)

### Neck/Head (16 muscles)
- Longissimus_Capitis, Longus_Capitis, Omohyoid, Platysma,
  Scalene Anterior/Middle, Semispinalis_Capitis, Splenius Capitis/Cervicis,
  Sternocleidomastoid

### Trunk-to-Arm (26 muscles - likely KEPT by paper)
Muscles connecting trunk to shoulder/arm:
- Coracobrachialis, Deltoid (3 heads), Infraspinatus, Latissimus_Dorsi (3 heads),
  Pectoralis_Major/Minor, Rhomboid_Major/Minor, Serratus_Anterior,
  Subclavian, Subscapularis, Supraspinatus, Teres_Major, Trapezius (3 heads)

---

## DOF Index Reference (for ExoNN)

| DOF | Joint | Body | Description |
|-----|-------|------|-------------|
| 6 | FemurR (Ball, X) | Right hip | **Flexion/Extension** |
| 7 | FemurR (Ball, Y) | Right hip | Abduction/Adduction |
| 8 | FemurR (Ball, Z) | Right hip | Internal/External rotation |
| 15 | FemurL (Ball, X) | Left hip | **Flexion/Extension** |
| 16 | FemurL (Ball, Y) | Left hip | Abduction/Adduction |
| 17 | FemurL (Ball, Z) | Left hip | Internal/External rotation |

ExoNN reads DOFs 6 and 15 (positions and velocities) for the 16-dim input.
Active DOF indices (excl root): FemurR flex/ext = 0, FemurL flex/ext = 9.

---

## Recommended Muscle Subsets for Implementation

### 208-Muscle Subset (for training)
**Rule:** Include all muscles EXCEPT pure arm/hand (62) and neck/head (16) = 78 excluded.
This gives 206 muscles. To reach 208, also include 2 trunk-only muscles
(L/R_Multifidus or L/R_iliocostalis).

### 108 Lower-Limb Subset (for exo reward r_m)
**Rule:** Include all hip-crossing muscles (92) + selected knee extensors/flexors (16):
- All 92 hip-crossing muscles
- Vastus Intermedius/Lateralis/Medialis (12 muscles) - primary knee extensors
- Gastrocnemius Lateral/Medial Head (4 muscles) - cross knee and ankle
- Total: 108

These are the muscles most affected by hip exoskeleton assistance and most
relevant to the metabolic cost reduction measured in the paper.
