/*
 * Copyright (C) 2024 libass contributors
 *
 * This file is part of libass.
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * Unit tests for the VSFilter-compatible tokenisation of \p vector drawings.
 *
 * Every expectation below was derived by hand from the parsing rules used
 * by modern VSFilter (xy-VSFilter / MPC-HC ISR):
 *   1. Unknown/invalid commands are ignored together with all coordinates
 *      following them (the coordinates are not appended to the preceding
 *      valid command).
 *   2. n is accepted as the first node if an (invalid) m appeared earlier;
 *      n before any m yields no root node (empty outline).
 *   3. b greedily collects points in triplets; points not forming a
 *      complete triplet are discarded.
 *   4. s commands with fewer than three points are ignored entirely.
 *   5. p commands are ignored while fewer than three nodes exist.
 *   6. There is no q (conic bezier) command: q and its coordinates are
 *      handled like any other invalid command.
 *
 * The tests compare the full parsed ASS_Outline (control point coordinates
 * and segment tags) against hand-computed expectations, which also covers
 * the internal cubic-B-spline to cubic-Bezier conversion.
 *
 * Outline storage reminder (see ass_outline.h): a segment owns as many
 * points as its order (line = 1, cubic = 3) and uses the first point of
 * the next segment as its endpoint; moves do not create segments. Each
 * parsed drawing coordinate therefore becomes one outline point, while the
 * segment count is lower. A contour is closed by OR-ing OUTLINE_CONTOUR_END
 * into its final segment tag.
 */

#include "config.h"
#include "ass_compat.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ass_utils.h"
#include "ass_drawing.h"
#include "ass_outline.h"
#include "ass_bitmap.h"
#include "ass_bitmap_engine.h"
#include "ass_rasterizer.h"

#define RASTERIZER_TEST_PRECISION 16

typedef struct {
    int32_t x, y;
} ExpPoint;

typedef struct {
    const char *text;       // drawing text fed to ass_drawing_parse
    const char *desc;       // human-readable description
    bool parse_ok;          // expected ass_drawing_parse result
    const ExpPoint *points; // expected outline control points (in d6 units)
    size_t n_points;
    const char *segments;   // expected segment tags
    size_t n_segments;
} DrawingTest;

static int tests_run;
static int tests_failed;

// Contour end marker. ass_drawing_parse always closes a contour by adding
// an OUTLINE_LINE_SEGMENT tagged with OUTLINE_CONTOUR_END, regardless of
// the kind of the preceding segments, so the closing segment tag is always
// OUTLINE_LINE_SEGMENT | OUTLINE_CONTOUR_END == 5.
#define END_SEG  (char) (OUTLINE_LINE_SEGMENT | OUTLINE_CONTOUR_END)

static bool check_outline(const DrawingTest *tc,
                          const ASS_Outline *ol, bool ok)
{
    bool pass = true;

    if (ok != tc->parse_ok) {
        printf("    FAIL: parse result %s, expected %s\n",
               ok ? "success" : "failure",
               tc->parse_ok ? "success" : "failure");
        return false;
    }
    if (!ok)
        return true;

    if (ol->n_points != tc->n_points) {
        printf("    FAIL: n_points = %zu, expected %zu\n",
               ol->n_points, tc->n_points);
        pass = false;
    }
    if (ol->n_segments != tc->n_segments) {
        printf("    FAIL: n_segments = %zu, expected %zu\n",
               ol->n_segments, tc->n_segments);
        pass = false;
    }

    size_t n_points = ol->n_points < tc->n_points
                    ? ol->n_points : tc->n_points;
    for (size_t i = 0; i < n_points; i++) {
        if (ol->points[i].x != tc->points[i].x ||
            ol->points[i].y != tc->points[i].y) {
            printf("    FAIL: point %zu = (%d, %d), expected (%d, %d)\n",
                   i, ol->points[i].x, ol->points[i].y,
                   tc->points[i].x, tc->points[i].y);
            pass = false;
        }
    }

    size_t n_segments = ol->n_segments < tc->n_segments
                      ? ol->n_segments : tc->n_segments;
    for (size_t i = 0; i < n_segments; i++) {
        if (ol->segments[i] != tc->segments[i]) {
            printf("    FAIL: segment %zu = %d, expected %d\n",
                   i, (int) (unsigned char) ol->segments[i],
                   (int) (unsigned char) tc->segments[i]);
            pass = false;
        }
    }

    return pass;
}

static void run_test(const DrawingTest *tc)
{
    ASS_Outline ol;
    ASS_Rect cbox;
    memset(&ol, 0, sizeof(ol));

    bool ok = ass_drawing_parse(&ol, &cbox, tc->text, NULL);
    tests_run++;
    bool pass = check_outline(tc, &ol, ok);
    if (pass) {
        printf("  ok %u: %s\n", (unsigned) tests_run, tc->desc);
    } else {
        tests_failed++;
        printf("  FAILED: %s\n    input: \"%s\"\n", tc->desc, tc->text);
    }

    if (ok)
        ass_outline_free(&ol);
}

/*
 * Rasterise an outline into a freshly allocated bitmap.
 * The window starts at the outline bbox origin and is enlarged to tile
 * multiples, so identical outlines always yield byte-identical bitmaps.
 * The caller owns the returned bitmap and must release it with
 * ass_free_bitmap.
 */
static bool rasterize(const BitmapEngine *engine, const ASS_Outline *ol,
                      Bitmap *bm)
{
    RasterizerData rst;
    if (!ass_rasterizer_init(engine, &rst, RASTERIZER_TEST_PRECISION))
        return false;
    if (!ass_rasterizer_set_outline(&rst, ol, false)) {
        ass_rasterizer_done(&rst);
        return false;
    }

    int32_t tile = 1 << engine->tile_order;
    int32_t w = rst.bbox.x_max - rst.bbox.x_min + 1;
    int32_t h = rst.bbox.y_max - rst.bbox.y_min + 1;
    w = (w + tile - 1) & ~(tile - 1);
    h = (h + tile - 1) & ~(tile - 1);

    if (!ass_alloc_bitmap(engine, bm, w, h, true)) {
        ass_rasterizer_done(&rst);
        return false;
    }
    bool filled = ass_rasterizer_fill(engine, &rst, bm->buffer,
                                      rst.bbox.x_min, rst.bbox.y_min,
                                      bm->w, bm->h, bm->stride);
    ass_rasterizer_done(&rst);
    if (!filled) {
        ass_free_bitmap(bm);
        return false;
    }
    return true;
}

/*
 * Parse and rasterise two drawing strings and compare the resulting
 * bitmaps byte by byte. Used for the regression checks of well-formed
 * drawings.
 */
static void compare_render(const char *ref_text, const char *new_text,
                           const char *desc)
{
    tests_run++;
    BitmapEngine engine = ass_bitmap_engine_init(0);
    ASS_Outline ref_ol, new_ol;
    ASS_Rect ref_cbox, new_cbox;
    memset(&ref_ol, 0, sizeof(ref_ol));
    memset(&new_ol, 0, sizeof(new_ol));

    bool pass = false;
    Bitmap ref_bm = {0}, new_bm = {0};
    bool ref_rasterized = false, new_rasterized = false;

    bool ref_ok = ass_drawing_parse(&ref_ol, &ref_cbox, ref_text, NULL);
    bool new_ok = ass_drawing_parse(&new_ol, &new_cbox, new_text, NULL);
    if (ref_ok != new_ok) {
        printf("  FAILED: %s: parse result mismatch\n", desc);
        goto out;
    }
    if (!ref_ok) {
        pass = true;  // both rejected: identical behaviour
        goto out;
    }

    if (!rasterize(&engine, &ref_ol, &ref_bm)) {
        printf("  FAILED: %s: rasterisation failure\n", desc);
        goto out;
    }
    ref_rasterized = true;
    if (!rasterize(&engine, &new_ol, &new_bm)) {
        printf("  FAILED: %s: rasterisation failure\n", desc);
        goto out;
    }
    new_rasterized = true;

    size_t ref_size = (size_t) ref_bm.stride * ref_bm.h;
    size_t new_size = (size_t) new_bm.stride * new_bm.h;
    if (ref_bm.w != new_bm.w || ref_bm.h != new_bm.h ||
        ref_size != new_size || memcmp(ref_bm.buffer, new_bm.buffer, ref_size)) {
        printf("  FAILED: %s: rendered bitmaps differ\n", desc);
        goto out;
    }
    pass = true;

out:
    if (ref_rasterized)
        ass_free_bitmap(&ref_bm);
    if (new_rasterized)
        ass_free_bitmap(&new_bm);
    if (ref_ok)
        ass_outline_free(&ref_ol);
    if (new_ok)
        ass_outline_free(&new_ol);
    if (pass)
        printf("  ok %u: %s\n", (unsigned) tests_run, desc);
    else
        tests_failed++;
}

#define L(x, y) (ExpPoint) { (int32_t) ((x) * 64), (int32_t) ((y) * 64) }

int main(void)
{
    /*
     * ---- Rule 1: invalid commands are skipped with all their coords ----
     */
    {
        // 'z' is unknown; its two coords must not be appended to the move,
        // and the following line is still parsed normally.
        // Tokens: M(0,0) L(10,10) L(0,10) -> 3 points; two line segments
        // plus the contour-closing line segment.
        static const ExpPoint pts[] = {
            L(0, 0), L(10, 10), L(0, 10),
        };
        static const char segs[] = { 1, 1, END_SEG };
        static const DrawingTest tc = {
            "m 0 0 z 5 5 l 10 10 0 10",
            "invalid command z: coords ignored, not merged into m/l",
            true, pts, sizeof(pts) / sizeof(pts[0]),
            segs, sizeof(segs)
        };
        run_test(&tc);
    }
    {
        // Invalid command right after the initial move: the orphan coords
        // are dropped, subsequent line works.
        static const ExpPoint pts[] = {
            L(1, 2), L(3, 4),
        };
        static const char segs[] = { 1, END_SEG };
        static const DrawingTest tc = {
            "m 1 2 X 9 9 l 3 4",
            "invalid command X after move: orphan coords dropped",
            true, pts, sizeof(pts) / sizeof(pts[0]),
            segs, sizeof(segs)
        };
        run_test(&tc);
    }
    {
        // An invalid command may carry any number of coords; all of them
        // are consumed until the next known command letter.
        static const ExpPoint pts[] = {
            L(0, 0), L(6, 7),
        };
        static const char segs[] = { 1, END_SEG };
        static const DrawingTest tc = {
            "m 0 0 ? 1 2 3 4 5 6 l 6 7",
            "invalid command ?: all following coords consumed and ignored",
            true, pts, sizeof(pts) / sizeof(pts[0]),
            segs, sizeof(segs)
        };
        run_test(&tc);
    }
    {
        // Only invalid commands: no root node, parse succeeds with an
        // empty outline.
        static const DrawingTest tc = {
            "z 1 2 z 3 4",
            "only invalid commands: empty outline",
            true, NULL, 0, NULL, 0
        };
        run_test(&tc);
    }
    {
        // An invalid command between two line groups cannot smuggle its
        // coordinates into either group.
        static const ExpPoint pts[] = {
            L(0, 0), L(1, 0), L(2, 0),
        };
        static const char segs[] = { 1, 1, END_SEG };
        static const DrawingTest tc = {
            "m 0 0 l 1 0 Z 9 9 l 2 0",
            "invalid command between line groups keeps groups separate",
            true, pts, sizeof(pts) / sizeof(pts[0]),
            segs, sizeof(segs)
        };
        run_test(&tc);
    }

    /*
     * ---- Rule 6: q does not exist and is treated as invalid ----
     */
    {
        static const ExpPoint pts[] = {
            L(0, 0), L(4, 5),
        };
        static const char segs[] = { 1, END_SEG };
        static const DrawingTest tc = {
            "m 0 0 q 1 2 3 4 l 4 5",
            "q command unknown: q and its coordinates ignored",
            true, pts, sizeof(pts) / sizeof(pts[0]),
            segs, sizeof(segs)
        };
        run_test(&tc);
    }
    {
        // q before any valid command cannot establish a root node.
        static const ExpPoint pts[] = {
            L(7, 8), L(9, 0),
        };
        static const char segs[] = { 1, END_SEG };
        static const DrawingTest tc = {
            "q 1 2 3 4 m 7 8 l 9 0",
            "q before root: ignored, m establishes root",
            true, pts, sizeof(pts) / sizeof(pts[0]),
            segs, sizeof(segs)
        };
        run_test(&tc);
    }

    /*
     * ---- Rule 2: n can be the first node only after an invalid m ----
     */
    {
        // m without coords is invalid, then n establishes the root.
        static const ExpPoint pts[] = {
            L(2, 3), L(4, 5),
        };
        static const char segs[] = { 1, END_SEG };
        static const DrawingTest tc = {
            "m n 2 3 l 4 5",
            "invalid m then n: n becomes root node",
            true, pts, sizeof(pts) / sizeof(pts[0]),
            segs, sizeof(segs)
        };
        run_test(&tc);
    }
    {
        // m with only one coordinate is invalid; n becomes root.
        static const ExpPoint pts[] = {
            L(6, 6), L(7, 7),
        };
        static const char segs[] = { 1, END_SEG };
        static const DrawingTest tc = {
            "m 1 n 6 6 l 7 7",
            "m with missing y then n: n becomes root node",
            true, pts, sizeof(pts) / sizeof(pts[0]),
            segs, sizeof(segs)
        };
        run_test(&tc);
    }
    {
        // n before any m (valid or invalid) yields an empty outline.
        static const DrawingTest tc = {
            "n 1 2 l 3 4",
            "n before any m: no root node, empty outline",
            true, NULL, 0, NULL, 0
        };
        run_test(&tc);
    }
    {
        // An invalid non-m command before n does not count: still no root.
        static const DrawingTest tc = {
            "z 0 0 n 1 2 l 3 4",
            "n after invalid non-m command: no root, empty outline",
            true, NULL, 0, NULL, 0
        };
        run_test(&tc);
    }
    {
        // After a valid m, n is a non-closed move: no contour end is
        // emitted there. Tokens M(0,0) L(1,1) NC(2,2) L(3,3); the single
        // contour is closed at the end -> 3 points, 3 line segments.
        static const ExpPoint pts[] = {
            L(0, 0), L(1, 1), L(3, 3),
        };
        static const char segs[] = { 1, 1, END_SEG };
        static const DrawingTest tc = {
            "m 0 0 l 1 1 n 2 2 l 3 3",
            "n after valid m: move without closing contour",
            true, pts, sizeof(pts) / sizeof(pts[0]),
            segs, sizeof(segs)
        };
        run_test(&tc);
    }

    /*
     * ---- Rule 3: b drops points not part of a triplet ----
     */
    {
        // 3 points: one complete cubic bezier.
        static const ExpPoint pts[] = {
            L(0, 0), L(1, 1), L(2, 2), L(3, 0),
        };
        static const char segs[] = { 3, END_SEG };
        static const DrawingTest tc = {
            "m 0 0 b 1 1 2 2 3 0",
            "b with exactly 3 points: one cubic bezier",
            true, pts, sizeof(pts) / sizeof(pts[0]),
            segs, sizeof(segs)
        };
        run_test(&tc);
    }
    {
        // 4 points: triplet kept, the trailing single point dropped.
        static const ExpPoint pts[] = {
            L(0, 0), L(1, 1), L(2, 2), L(3, 0),
        };
        static const char segs[] = { 3, END_SEG };
        static const DrawingTest tc = {
            "m 0 0 b 1 1 2 2 3 0 4 4",
            "b with 4 points: trailing point dropped",
            true, pts, sizeof(pts) / sizeof(pts[0]),
            segs, sizeof(segs)
        };
        run_test(&tc);
    }
    {
        // 5 points: triplet kept, the trailing pair dropped.
        static const ExpPoint pts[] = {
            L(0, 0), L(1, 1), L(2, 2), L(3, 0),
        };
        static const char segs[] = { 3, END_SEG };
        static const DrawingTest tc = {
            "m 0 0 b 1 1 2 2 3 0 4 4 5 5",
            "b with 5 points: trailing pair dropped",
            true, pts, sizeof(pts) / sizeof(pts[0]),
            segs, sizeof(segs)
        };
        run_test(&tc);
    }
    {
        // 2 points: no triplet at all. The failed command adds nothing,
        // and since no segment is ever created the move yields no point.
        static const DrawingTest tc = {
            "m 0 0 b 1 1 2 2",
            "b with 2 points: whole command ignored, empty outline",
            true, NULL, 0, NULL, 0
        };
        run_test(&tc);
    }
    {
        // 7 points: two triplets (two beziers), one trailing point dropped.
        static const ExpPoint pts[] = {
            L(0, 0), L(1, 1), L(2, 2), L(3, 3),
            L(4, 4), L(5, 5), L(6, 6),
        };
        static const char segs[] = { 3, 3, END_SEG };
        static const DrawingTest tc = {
            "m 0 0 b 1 1 2 2 3 3 4 4 5 5 6 6 7 7",
            "b with 7 points: two triplets kept, one point dropped",
            true, pts, sizeof(pts) / sizeof(pts[0]),
            segs, sizeof(segs)
        };
        run_test(&tc);
    }
    {
        // A dangling coordinate after b is not picked up by the following
        // l either; l starts reading after its command letter.
        static const ExpPoint pts[] = {
            L(0, 0), L(9, 9),
        };
        static const char segs[] = { 1, END_SEG };
        static const DrawingTest tc = {
            "m 0 0 b 1 1 l 9 9",
            "b with 1 point drops it; following l is unaffected",
            true, pts, sizeof(pts) / sizeof(pts[0]),
            segs, sizeof(segs)
        };
        run_test(&tc);
    }

    /*
     * ---- Rule 4: s with fewer than three points is ignored ----
     */
    {
        static const DrawingTest tc = {
            "m 0 0 s 1 1",
            "s with 1 point: ignored, empty outline",
            true, NULL, 0, NULL, 0
        };
        run_test(&tc);
    }
    {
        static const DrawingTest tc = {
            "m 0 0 s 1 1 2 2",
            "s with 2 points: ignored, empty outline",
            true, NULL, 0, NULL, 0
        };
        run_test(&tc);
    }
    {
        // A failed s must not leave spline state behind, so a later c
        // has no effect.
        static const ExpPoint pts[] = {
            L(0, 0), L(8, 8),
        };
        static const char segs[] = { 1, END_SEG };
        static const DrawingTest tc = {
            "m 0 0 s 1 1 c l 8 8",
            "failed s then c: c has no spline to close",
            true, pts, sizeof(pts) / sizeof(pts[0]),
            segs, sizeof(segs)
        };
        run_test(&tc);
    }
    {
        // Well-formed s with three points: one B-spline segment converted
        // to a cubic bezier. Token nodes p0=(0,0) S(3,0) S(6,0) S(9,0),
        // integer d6 deltas all equal 64:
        //   out0 = p1 + (64-64)/2 = (192,0)
        //   out1 = p1 + 64        = (256,0)
        //   out2 = p2 - 64        = (320,0)
        //   out3 = p2 + (64-64)/2 = (384,0)
        static const ExpPoint pts[] = {
            { 192, 0 }, { 256, 0 }, { 320, 0 }, { 384, 0 },
        };
        static const char segs[] = { 3, END_SEG };
        static const DrawingTest tc = {
            "m 0 0 s 3 0 6 0 9 0",
            "s with 3 points: converted to one cubic bezier",
            true, pts, sizeof(pts) / sizeof(pts[0]),
            segs, sizeof(segs)
        };
        run_test(&tc);
    }

    /*
     * ---- Rule 5: p is ignored while fewer than three nodes exist ----
     */
    {
        // Only the move node exists (1 node): p is ignored.
        static const DrawingTest tc = {
            "m 0 0 p 1 1 2 2 3 3",
            "p with only 1 preceding node: ignored, empty outline",
            true, NULL, 0, NULL, 0
        };
        run_test(&tc);
    }
    {
        // Two nodes (move + line): p is still ignored.
        static const ExpPoint pts[] = {
            L(0, 0), L(1, 1),
        };
        static const char segs[] = { 1, END_SEG };
        static const DrawingTest tc = {
            "m 0 0 l 1 1 p 2 2 3 3",
            "p with only 2 preceding nodes: ignored",
            true, pts, sizeof(pts) / sizeof(pts[0]),
            segs, sizeof(segs)
        };
        run_test(&tc);
    }
    {
        // A valid s establishes three spline nodes (4 total including the
        // move), so a following p extends the spline.
        // First bezier (from the s): (192,0)(256,0)(320,0)(384,0).
        // The extension uses the three preceding TOKEN nodes
        // S(3,0)=(192) S(6,0)=(384) S(9,0)=(576) together with P(12,0)=768;
        // deltas 64 each, so:
        //   out0 = 384+0 = 384 (== prior bezier end, not re-added)
        //   out1 = 384+64 = 448; out2 = 576-64 = 512; out3 = 576+0 = 576.
        // Only out1..out3 are new outline points (out0 is the shared end).
        static const ExpPoint pts[] = {
            { 192, 0 }, { 256, 0 }, { 320, 0 }, { 384, 0 },
            { 448, 0 }, { 512, 0 }, { 576, 0 },
        };
        static const char segs[] = { 3, 3, END_SEG };
        static const DrawingTest tc = {
            "m 0 0 s 3 0 6 0 9 0 p 12 0",
            "p after valid s (>=3 nodes): extends the spline",
            true, pts, sizeof(pts) / sizeof(pts[0]),
            segs, sizeof(segs)
        };
        run_test(&tc);
    }
    {
        // Three nodes without any spline: p is accepted as an extension
        // spline node. The extension uses token nodes
        // M(0,0) L(1,0) L(2,0) P(3,0), integer-truncated deltas 64/3=21:
        //   out0 = 64 + (21-21)/2 = 64  (== line end, not re-added)
        //   out1 = 64 + 21  = 85
        //   out2 = 128 - 21 = 107
        //   out3 = 128 + (21-21)/2 = 128
        // Only out1..out3 are new outline points.
        static const ExpPoint pts[] = {
            L(0, 0), L(1, 0), L(2, 0),
            { 85, 0 }, { 107, 0 }, { 128, 0 },
        };
        static const char segs[] = { 1, 1, 3, END_SEG };
        static const DrawingTest tc = {
            "m 0 0 l 1 0 2 0 p 3 0",
            "p with 3 preceding nodes: extension spline accepted",
            true, pts, sizeof(pts) / sizeof(pts[0]),
            segs, sizeof(segs)
        };
        run_test(&tc);
    }

    /*
     * ---- Combined / interaction cases ----
     */
    {
        // Invalid command between two b groups; the second b starts fresh.
        static const ExpPoint pts[] = {
            L(0, 0), L(1, 1), L(2, 2), L(3, 3),
            L(4, 4), L(5, 5), L(6, 6),
        };
        static const char segs[] = { 3, 3, END_SEG };
        static const DrawingTest tc = {
            "m 0 0 b 1 1 2 2 3 3 Z 0 0 b 4 4 5 5 6 6",
            "invalid command between two b groups does not merge coords",
            true, pts, sizeof(pts) / sizeof(pts[0]),
            segs, sizeof(segs)
        };
        run_test(&tc);
    }
    {
        // Whitespace handling and a trailing incomplete b triplet.
        static const ExpPoint pts[] = {
            L(0, 0), L(1, 0), L(2, 0), L(3, 0),
        };
        static const char segs[] = { 3, END_SEG };
        static const DrawingTest tc = {
            "  m   0 0  b 1 0 2 0 3 0 4 0 ",
            "extra whitespace tolerated; dangling triplet remainder dropped",
            true, pts, sizeof(pts) / sizeof(pts[0]),
            segs, sizeof(segs)
        };
        run_test(&tc);
    }
    {
        // Two consecutive s commands: the second s starts a new spline
        // whose first bezier shares the previous spline's last token node
        // (9,0) as its p0 anchor.
        // First bezier: (192,0)(256,0)(320,0)(384,0).
        // Second bezier over tokens (9,0)(12,0)(15,0)(18,0):
        //   p1=768, p2=960, p3=1152 (d6); deltas 192/3=64
        //   out0 = 768 + 0 = 768 (== prior bezier end, not re-added)
        //   out1 = 768+64 = 832; out2 = 960-64 = 896; out3 = 960+0 = 960.
        // Only out1..out3 are new outline points.
        static const ExpPoint pts[] = {
            { 192, 0 }, { 256, 0 }, { 320, 0 }, { 384, 0 },
            { 832, 0 }, { 896, 0 }, { 960, 0 },
        };
        static const char segs[] = { 3, 3, END_SEG };
        static const DrawingTest tc = {
            "m 0 0 s 3 0 6 0 9 0 s 12 0 15 0 18 0",
            "second s starts a new independent spline",
            true, pts, sizeof(pts) / sizeof(pts[0]),
            segs, sizeof(segs)
        };
        run_test(&tc);
    }

    /*
     * ---- Pixel-level regression: well-formed drawings ----
     * These inputs use only well-formed commands; the tokeniser must
     * produce stable outlines and byte-identical rendered bitmaps.
     */
    compare_render(
        "m 0 0 l 100 0 100 100 0 100",
        "m 0 0 l 100 0 100 100 0 100",
        "regression: simple rectangle renders deterministically");
    compare_render(
        "m 0 0 b 20 0 80 0 100 20 b 100 80 80 100 0 100 b 20 100 0 80 0 20",
        "m 0 0 b 20 0 80 0 100 20 b 100 80 80 100 0 100 b 20 100 0 80 0 20",
        "regression: closed cubic-bezier contour renders deterministically");
    compare_render(
        "m 0 0 s 10 0 20 0 30 0 p 40 0 50 0 c l 0 50",
        "m 0 0 s 10 0 20 0 30 0 p 40 0 50 0 c l 0 50",
        "regression: closed b-spline contour renders deterministically");
    compare_render(
        "m 0 0 l 50 0 m 10 10 l 60 10 60 60 10 60",
        "m 0 0 l 50 0 m 10 10 l 60 10 60 60 10 60",
        "regression: multiple contours render deterministically");
    compare_render(
        "m 0 0 n 10 10 l 40 40 n 0 0 l 50 50",
        "m 0 0 n 10 10 l 40 40 n 0 0 l 50 50",
        "regression: n (move-noclose) contours render deterministically");

    printf("\n%u tests, %d failures\n", tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}
