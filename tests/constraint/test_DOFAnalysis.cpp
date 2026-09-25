// The degrees-of-freedom analysis behind a sketch's colours (Phase 138). It
// took the whole Jacobian dense and its SVD, after every edit: for a large
// sketch hundreds of megabytes and seconds. Now each cluster of parameters
// the constraints tie together is analysed alone, from the Jacobian's
// non-zeros, and has its own status: an over-constrained corner no longer
// turns the whole sketch red, nor a free one the whole sketch green.

#include <gtest/gtest.h>

#include <Eigen/SVD>
#include <algorithm>
#include <chrono>
#include <memory>

#include "../TimeLimits.h"
#include "horizon/constraint/Constraint.h"
#include "horizon/constraint/ConstraintSystem.h"
#include "horizon/constraint/GeometryRef.h"
#include "horizon/constraint/ParameterTable.h"
#include "horizon/constraint/SketchSolver.h"
#include "horizon/drafting/DraftDocument.h"
#include "horizon/drafting/DraftLine.h"

using namespace hz;
using math::Vec2;

namespace {

cstr::GeometryRef point(uint64_t entity, int index) {
    return {entity, cstr::FeatureType::Point, index};
}

std::shared_ptr<draft::DraftLine> line(draft::DraftDocument& doc, Vec2 a, Vec2 b) {
    auto l = std::make_shared<draft::DraftLine>(a, b);
    doc.addEntity(l);
    return l;
}

/// The rank of the whole Jacobian, dense, as the analysis used to take it.
int denseRank(const cstr::ParameterTable& params, const cstr::ConstraintSystem& sys) {
    Eigen::MatrixXd j = Eigen::MatrixXd::Zero(sys.totalEquations(), params.parameterCount());
    int offset = 0;
    for (const auto& c : sys.constraints()) {
        c->jacobian(params, j, offset);
        offset += c->equationCount();
    }
    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(j);
    const auto& sigma = svd.singularValues();
    if (sigma.size() == 0 || sigma(0) <= 0.0) return 0;
    const double threshold = 1e-8 * static_cast<double>(std::max(j.rows(), j.cols())) * sigma(0);
    int rank = 0;
    for (int i = 0; i < sigma.size(); ++i) rank += sigma(i) > threshold ? 1 : 0;
    return rank;
}

}  // namespace

// Three lines tied by nothing to one another: one pinned down, one only kept
// horizontal, one kept horizontal twice. Each has its own status. The whole
// sketch was over-constrained, every line of it red.
TEST(DOFAnalysisTest, EachClusterHasItsOwnStatus) {
    draft::DraftDocument doc;
    auto pinned = line(doc, Vec2(0, 0), Vec2(10, 0));
    auto loose = line(doc, Vec2(0, 5), Vec2(10, 5));
    auto twice = line(doc, Vec2(0, 10), Vec2(10, 10));
    cstr::ConstraintSystem sys;
    sys.addConstraint(std::make_shared<cstr::FixedConstraint>(point(pinned->id(), 0), Vec2(0, 0)));
    sys.addConstraint(std::make_shared<cstr::FixedConstraint>(point(pinned->id(), 1), Vec2(10, 0)));
    sys.addConstraint(
        std::make_shared<cstr::HorizontalConstraint>(point(loose->id(), 0), point(loose->id(), 1)));
    for (int k = 0; k < 2; ++k) {
        sys.addConstraint(std::make_shared<cstr::HorizontalConstraint>(point(twice->id(), 0),
                                                                       point(twice->id(), 1)));
    }
    const auto params = cstr::ParameterTable::buildFromEntities(doc.entities(), sys);
    const auto analysis = cstr::SketchSolver().analyzeDOF(params, sys);

    EXPECT_EQ(analysis.entityStatus.at(pinned->id()), cstr::EntityDOFStatus::FullyConstrained);
    EXPECT_EQ(analysis.entityStatus.at(loose->id()), cstr::EntityDOFStatus::Free);
    EXPECT_EQ(analysis.entityStatus.at(twice->id()), cstr::EntityDOFStatus::OverConstrained);
    EXPECT_EQ(analysis.totalDOF, params.parameterCount() - denseRank(params, sys));
    EXPECT_EQ(analysis.totalDOF, 3 + 3) << "the loose and the doubled line keep three each";
}

// The freedom left is what the whole Jacobian's rank says, for chains of
// every length and mix of constraints, joined or not: the clusters add up.
TEST(DOFAnalysisTest, TheFreedomAgreesWithTheWholeJacobian) {
    for (int lines = 1; lines <= 24; ++lines) {
        draft::DraftDocument doc;
        cstr::ConstraintSystem sys;
        std::vector<std::shared_ptr<draft::DraftLine>> chain;
        for (int i = 0; i < lines; ++i) {
            chain.push_back(line(doc, Vec2(i * 10.0, (i % 3) * 1.0), Vec2(i * 10.0 + 8, i * 2.0)));
        }
        for (int i = 0; i < lines; ++i) {
            const uint64_t id = chain[static_cast<size_t>(i)]->id();
            // Joined to the next, except every fifth: several clusters.
            if (i + 1 < lines && i % 5 != 4) {
                sys.addConstraint(std::make_shared<cstr::CoincidentConstraint>(
                    point(id, 1), point(chain[static_cast<size_t>(i + 1)]->id(), 0)));
            }
            if (i % 2 == 0) {
                sys.addConstraint(
                    std::make_shared<cstr::HorizontalConstraint>(point(id, 0), point(id, 1)));
            }
            if (i % 3 == 0) {
                sys.addConstraint(
                    std::make_shared<cstr::DistanceConstraint>(point(id, 0), point(id, 1), 8.0));
            }
            if (i % 7 == 0) {
                sys.addConstraint(std::make_shared<cstr::FixedConstraint>(point(id, 0), Vec2()));
            }
        }
        const auto params = cstr::ParameterTable::buildFromEntities(doc.entities(), sys);
        const auto analysis = cstr::SketchSolver().analyzeDOF(params, sys);
        EXPECT_EQ(analysis.totalDOF, params.parameterCount() - denseRank(params, sys))
            << lines << " lines";
    }
}

// A large sketch is analysed quickly: two thousand lines each held on its
// own, and a chain of a thousand joined end to start. The whole Jacobian,
// dense, would be 9,000 x 12,000 (860 MB); a chain of 400 lines alone took
// 33 s that way in a Debug build, and takes 7 ms now.
TEST(DOFAnalysisTest, ALargeSketchIsAnalysedQuickly) {
    HZ_SKIP_WITHOUT_TIME_LIMITS();
    draft::DraftDocument doc;
    cstr::ConstraintSystem sys;
    for (int i = 0; i < 2000; ++i) {
        auto l = line(doc, Vec2(i * 3.0, 0), Vec2(i * 3.0 + 2, 1));
        sys.addConstraint(
            std::make_shared<cstr::FixedConstraint>(point(l->id(), 0), Vec2(i * 3.0, 0)));
        sys.addConstraint(
            std::make_shared<cstr::HorizontalConstraint>(point(l->id(), 0), point(l->id(), 1)));
    }
    std::shared_ptr<draft::DraftLine> previous;
    for (int i = 0; i < 1000; ++i) {
        auto l = line(doc, Vec2(i * 4.0, 100), Vec2(i * 4.0 + 4, 100 + (i % 2)));
        if (previous) {
            sys.addConstraint(std::make_shared<cstr::CoincidentConstraint>(point(previous->id(), 1),
                                                                           point(l->id(), 0)));
        }
        if (i % 2 == 0) {
            sys.addConstraint(
                std::make_shared<cstr::HorizontalConstraint>(point(l->id(), 0), point(l->id(), 1)));
        } else {
            sys.addConstraint(
                std::make_shared<cstr::VerticalConstraint>(point(l->id(), 0), point(l->id(), 1)));
        }
        previous = l;
    }
    const auto params = cstr::ParameterTable::buildFromEntities(doc.entities(), sys);
    const auto start = std::chrono::steady_clock::now();
    const auto analysis = cstr::SketchSolver().analyzeDOF(params, sys);
    const double ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();

    // Each held line has its end's x free: one each. The chain: each line
    // loses one to its direction and two to its joint.
    EXPECT_EQ(analysis.totalDOF, 2000 + 4000 - 1000 - 2 * 999);
    EXPECT_EQ(analysis.entityStatus.size(), 3000u);
#ifdef NDEBUG
    EXPECT_LT(ms, 150.0);
#else
    // 84 ms here; CI's Windows runner, with four tests at once, is slower.
    EXPECT_LT(ms, 3000.0);
#endif
}
