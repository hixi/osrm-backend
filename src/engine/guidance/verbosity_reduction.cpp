#include "engine/guidance/verbosity_reduction.hpp"
#include "engine/guidance/collapsing_utility.hpp"

#include <boost/assert.hpp>
#include <iterator>

namespace osrm
{
namespace engine
{
namespace guidance
{
std::vector<RouteStep> suppressShortNameSegments(std::vector<RouteStep> steps)
{
    // guard against empty routes, even though they shouldn't happen
    if (steps.empty())
        return steps;

    BOOST_ASSERT(!hasTurnType(steps.back()) && hasWaypointType(steps.back()));
    for (auto prev = steps.begin(), itr = std::next(prev); !hasWaypointType(*itr); ++itr)
    {
        BOOST_ASSERT(hasTurnType(*itr) && !hasTurnType(*itr, TurnType::Suppressed));

        if (hasTurnType(*itr, TurnType::NewName) && haveSameMode(*prev, *itr) && !hasLanes(*itr))
        {
            const auto name = itr;
            if (haveSameName(*prev, *itr))
            {
                prev->ElongateBy(*itr);
                itr->Invalidate();
            }
            else
            {
                auto distance = itr->distance;
                // sum up all distances that can be relevant to the name change
                while (
                    !hasWaypointType(*(itr + 1)) &&
                    (!hasTurnType(*(itr + 1)) || hasTurnType(*(itr + 1), TurnType::Suppressed)) &&
                    distance < NAME_SEGMENT_CUTOFF_LENGTH)
                {
                    ++itr;
                    distance += itr->distance;
                }

                if (distance < NAME_SEGMENT_CUTOFF_LENGTH)
                {
                    prev->ElongateBy(*itr);
                    itr->Invalidate();
                }
                else
                    prev = name;
            }
        }
        else
        {
            // remember last item
            prev = itr;
        }
    }
    return removeNoTurnInstructions(std::move(steps));
}

// `useLane` steps are only returned on `straight` maneuvers when there
// are surrounding lanes also tagged as `straight`. If there are no other `straight`
// lanes, it is not an ambiguous maneuver, and we can collapse the `useLane` step.
std::vector<RouteStep> collapseUseLane(std::vector<RouteStep> steps)
{
    const auto containsTag = [](const extractor::guidance::TurnLaneType::Mask mask,
                                const extractor::guidance::TurnLaneType::Mask tag) {
        return (mask & tag) != extractor::guidance::TurnLaneType::empty;
    };

    const auto canCollapseUseLane = [containsTag](const RouteStep &step) {
        // the lane description is given left to right, lanes are counted from the right.
        // Therefore we access the lane description using the reverse iterator

        auto right_most_lanes = step.LanesToTheRight();
        if (!right_most_lanes.empty() && containsTag(right_most_lanes.front(),
                                                     (extractor::guidance::TurnLaneType::straight |
                                                      extractor::guidance::TurnLaneType::none)))
            return false;

        auto left_most_lanes = step.LanesToTheLeft();
        if (!left_most_lanes.empty() && containsTag(left_most_lanes.back(),
                                                    (extractor::guidance::TurnLaneType::straight |
                                                     extractor::guidance::TurnLaneType::none)))
            return false;

        return true;
    };

    const auto get_previous_index = [](std::size_t index, const std::vector<RouteStep> &steps) {
        BOOST_ASSERT(index > 0);
        BOOST_ASSERT(index < steps.size());
        --index;
        while (index > 0 && steps[index].maneuver.instruction.type == TurnType::NoTurn)
            --index;

        return index;
    };

    for (std::size_t step_index = 1; step_index < steps.size(); ++step_index)
    {
        const auto &step = steps[step_index];
        if (step.maneuver.instruction.type == TurnType::UseLane && canCollapseUseLane(step))
        {
            const auto previous = get_previous_index(step_index, steps);
            if (haveSameMode(steps[previous], step))
            {
                steps[previous].ElongateBy(steps[step_index]);
                steps[step_index].Invalidate();
            }
        }
    }
    return removeNoTurnInstructions(std::move(steps));
}

} // namespace guidance
} // namespace engine
} // namespace osrm
