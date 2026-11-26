#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "horizon/geometry/curves/Curve2D.h"
#include "horizon/geometry/curves/Curve3.h"
#include "horizon/geometry/surfaces/Surface.h"

namespace hz::geo {

/// A curve on a surface, in the surface's parameters: what a half-edge keeps of its edge.
struct Pcurve {
    /// Null exactly when no pcurve could be built at all (then @c problem says why and
    /// @c deviation is infinite); callers check @c problem first. A pcurve that was built but
    /// strays more than the tolerance is returned with @c problem set.
    std::shared_ptr<const Curve2D> curve;
    /// The pcurve's own parameter at the curve's t0.
    double start = 0.0;
    /// The pcurve's own parameter at the curve's t1.
    double end = 0.0;
    /// The largest distance measured between S(curve(s)) and the curve C.
    double deviation = 0.0;
    /// How far the curve itself stands off the surface: the largest distance measured from a
    /// point of it to the surface (for a closed form, its deviation; on the general path, its
    /// samples' gaps and their peaks between them, peakOfGap's search, where a peak could pass the
    /// tolerance), never above the deviation. No pcurve can come nearer the curve than this; a
    /// problem whose stand-off is above the tolerance is the curve's, not the fit's. Infinite when
    /// nothing was built.
    double standOff = 0.0;
    /// Empty when the pcurve lies within the tolerance; otherwise says why it does not.
    std::string problem;
};

/// The curve in @p surface's (u, v) that @p curve follows from @p t0 to @p t1, within
/// @p tolerance: continuous across a seam (u is unwrapped, so a rim over the seam runs past the
/// end of u's range), starting at t0's point and ending at t1's. A reversed range (t1 < t0)
/// gives a pcurve that runs the other way.
///
/// A curve that is a closed form on its surface (a line on a plane, a generator, a parallel, a
/// meridian, a circle or ellipse on a plane) gets the exact 2D curve, found geometrically and
/// confirmed by measuring it, never by the types alone: by pcurveGap, the validator's measure, at
/// 49 points and at the peaks of its gap between them, so that a tolerance grown to it is what the
/// validator finds. A planar surface's (u, v) is an affine map of the plane, so on a Plane, or on
/// a bilinear patch that is a parallelogram (as Horizon writes a planar face), a B-spline curve's
/// pcurve is its control points mapped into (u, v) with its weights and knots (a straight one of
/// two points is the Line2D between its ends): its
/// deviation is bounded from those control points' distances off the plane (and the patch's
/// departure from a parallelogram), so it is never below the true one, and the form is taken
/// only where that bound is within what is allowed. A cylinder's or cone's u window is the
/// start point's inverted u, wrapped into [0, 2 pi): a generator on the seam is at u = 0, or at
/// 2 pi where the start point inverts there (at a cone's apex, where u means nothing, the end
/// off the axis gives it). A meridian that starts at a pole takes the side its arc travels to,
/// so one that does not cross a pole stays in v's range; only a meridian through a pole
/// continues past it with v outside the range, which the surface still evaluates correctly.
/// A circle's sense is its turn about x cross y, so left-handed frames are read correctly.
///
/// Any other curve is sampled (adaptively, and again where the fit strays), inverted from
/// each previous point, and fitted by a cubic Nurbs2D by chord length, with up to 2048 samples.
/// Refinement is steered by a quick look inside each sample interval; the fit returned is
/// measured at seven points in each, its excess over the curve's own gap counted 3% more, and no
/// interval reads below the peak of its gap between those points (peakOfGap's search, where the
/// peak could pass the largest measured).
/// A curve that is farther than @p tolerance from the surface, or that the fit cannot follow
/// within it, comes back with @c problem set to say so (and the largest deviation measured).
///
/// @p standOffLimit is how far the curve may stand off the surface and still be taken (a
/// reader whose edge tolerances grow to cover the gaps in its file); at or below @p tolerance,
/// or not finite, nothing beyond the tolerance is. A curve measured to stand off by a gap g
/// within it is fitted to min(@p standOffLimit, 2 g + @p tolerance) rather than to
/// @p tolerance: no pcurve comes nearer the curve than its gap, so the gap is measured, not
/// chased (the factor 2 covers a gap that peaks between the samples that measured it; the peak
/// found between them is allowed for too when the fit is taken, and a curve whose gap peaks past
/// the limit between its samples comes back with its problem). A
/// closed form, whose deviation is its stand-off, is taken where that deviation is within the
/// limit.
Pcurve pcurveOn(const Surface& surface, const Curve3& curve, double t0, double t1, double tolerance,
                double standOffLimit = 0.0);

/// The largest of a gap sampled along a curve and of its peaks between the samples: how pcurveOn
/// and the exact validator find a gap's peak, so that a tolerance grown to what the one measures
/// is what the other checks (stage 2's lesson: two measures of one gap at different points part).
///
/// @p at holds the parameters sampled, in order along the curve (either way), @p gaps the gap at
/// each, and @p gapAt gives the gap at any parameter between them. Between each sample and its
/// neighbours where the gap rises to it (it is no lower than either; an end sample, than the one
/// beside it), the peak is searched for by golden section until it is pinned to @p rounding (the
/// rounding of what the gap is computed from), far finer than the 1% of its height that is asked
/// of it. Its height is how far it rises above the lower of its neighbours, or for an end sample
/// of the two samples on its inner side (the one beside it alone bounds nothing of a peak near the
/// far side of the end interval). The searches go highest first. One whose peak could not rise
/// past the largest gap found, nor past @p within (a peak that cannot pass it changes nothing its
/// caller decides), is not made: a peak is taken to rise at most its height again above its
/// sample, four times what a smooth one can above an inner sample and eight above an end one. A
/// gap that is not a number is passed over, and a sample beside one is not searched about. A peak
/// narrower than the samples' spacing, between two samples it does not lift, is not seen.
double peakOfGap(const std::vector<double>& at, const std::vector<double>& gaps,
                 const std::function<double(double)>& gapAt, double rounding, double within);

/// What pcurveGap measures.
struct PcurveGap {
    /// The farthest the pcurve, on its surface, strays from the curve: at its samples, and at the
    /// peaks between them that could pass the @c within it was measured with.
    double strays = 0.0;
    /// Each sample's nearest parameter on the curve (Curve3::closestParameter), in sample order.
    std::vector<double> params;
};

/// How far @p pcurve, mapped onto @p surface, strays from @p curve: at each of @p at (its own
/// parameters, in order along it, either way) the distance from its point on the surface to the
/// curve's nearest point (Curve3::closestParameter), and at the peaks between them (peakOfGap,
/// each point between measured by Newton's method from its neighbours' nearest points, and the
/// peak found measured as the samples are, the nearer of the two counting: each is a point of the
/// curve, so neither reads below the truth). Peaks that cannot pass @p within are not searched for
/// (what is measured is then within it). The one measure of a pcurve's gap from its curve: pcurveOn
/// measures a closed form by it, and topo::validateExact every pcurve.
PcurveGap pcurveGap(const Surface& surface, const Curve2D& pcurve, const std::vector<double>& at,
                    const Curve3& curve, double within);

}  // namespace hz::geo
