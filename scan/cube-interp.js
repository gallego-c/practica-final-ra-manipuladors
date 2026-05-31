/** Shared cube interpretation — used by index.html and calibrate.html */
const FACE_ORDER = ["U", "F", "R", "B", "L", "D"];

const CUBE_INTERPRET = {
    U: { scanKey: "U", remap: [2, 3, 0, 1] },
    F: { scanKey: "B", remap: [1, 0, 3, 2] },
    R: { scanKey: "R", remap: [1, 0, 3, 2] },
    B: { scanKey: "F", remap: [1, 0, 3, 2] },
    L: { scanKey: "L", remap: [1, 0, 3, 2] },
    D: { scanKey: "D", remap: [0, 2, 1, 3] }
};

function remapFaceColors(colors, remap) {
    return remap.map(i => colors[i]);
}

/** Physical cube face colors from raw scan captures (same logic as index.html). */
function getCubeFaceColors(cubeFace, faceData, live) {
    const { scanKey, remap } = CUBE_INTERPRET[cubeFace];
    let raw = null;
    if (live && live.scanStep === scanKey && live.activeColors) {
        raw = live.activeColors;
    } else if (faceData && faceData[scanKey]) {
        raw = faceData[scanKey];
    }
    return raw ? remapFaceColors(raw, remap) : null;
}

/** Calibrator mapping editor — same math, custom source/remap per slot. */
function getFaceColorsFromMapping(cubeFace, faceData, mapping, live) {
    const entry = mapping[cubeFace];
    if (!entry) return null;
    const scanKey = entry.source;
    let raw = null;
    if (live && live.scanStep === scanKey && live.activeColors) {
        raw = live.activeColors;
    } else if (faceData && faceData[scanKey]) {
        raw = faceData[scanKey];
    }
    return raw ? remapFaceColors(raw, entry.remap) : null;
}

function loadScannerPresetMapping() {
    const mapping = {};
    FACE_ORDER.forEach(face => {
        const { scanKey, remap } = CUBE_INTERPRET[face];
        mapping[face] = { source: scanKey, remap: [...remap] };
    });
    return mapping;
}
