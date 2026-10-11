// OwnerThread: what the modules and the in-test host rely on.
#include <gtest/gtest.h>

#include <atomic>
#include <memory>
#include <module_support/owner_thread.hpp>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

using module_support::OwnerThread;

TEST(OwnerThread, RunsTheClosureOnItsOwnThreadAndReturnsTheResult) {
    OwnerThread owner;
    const std::thread::id caller = std::this_thread::get_id();
    std::thread::id ran_on;
    const std::string result = owner.run([&] {
        ran_on = std::this_thread::get_id();
        return std::string("done");
    });
    EXPECT_EQ(result, "done");
    EXPECT_NE(ran_on, caller);
    EXPECT_FALSE(owner.onOwnerThread());
    // The same thread every time.
    std::thread::id again;
    owner.run([&] { again = std::this_thread::get_id(); });
    EXPECT_EQ(again, ran_on);
}

TEST(OwnerThread, ThrowsTheExceptionOfTheClosureInTheCaller) {
    OwnerThread owner;
    EXPECT_THROW(owner.run([] { throw std::runtime_error("failed"); }), std::runtime_error);
    EXPECT_THROW(owner.run([]() -> int { throw std::invalid_argument("bad"); }),
                 std::invalid_argument);
    // It keeps working afterwards.
    EXPECT_EQ(owner.run([] { return 7; }), 7);
}

TEST(OwnerThread, CallsAClosureHandedFromItsOwnThreadDirectly) {
    OwnerThread owner;
    const bool nested = owner.run([&] {
        EXPECT_TRUE(owner.onOwnerThread());
        return owner.run([&] { return owner.onOwnerThread(); });
    });
    EXPECT_TRUE(nested);
}

TEST(OwnerThread, SerializesCallersOfDifferentThreadsOnOneThread) {
    OwnerThread owner;
    std::atomic<int> inside{0};
    std::atomic<int> overlaps{0};
    std::vector<std::thread::id> threads(4 * 50);
    std::vector<std::thread> callers;
    for (int c = 0; c < 4; ++c) {
        callers.emplace_back([&, c] {
            for (int i = 0; i < 50; ++i) {
                owner.run([&] {
                    if (inside.fetch_add(1) != 0)
                        ++overlaps;
                    threads[static_cast<size_t>(c * 50 + i)] = std::this_thread::get_id();
                    std::this_thread::yield();
                    inside.fetch_sub(1);
                });
            }
        });
    }
    for (auto& caller : callers)
        caller.join();
    EXPECT_EQ(overlaps.load(), 0);
    for (const auto& id : threads)
        EXPECT_EQ(id, threads.front());
}

TEST(OwnerThread, StateWrittenByTheCallerIsSeenByTheOwnerAndTheOtherWayRound) {
    OwnerThread owner;
    int value = 1;  // plain memory: the hand-off orders the accesses
    for (int i = 0; i < 1000; ++i) {
        value += 1;
        owner.run([&] { value *= 2; });
        value -= 1;
    }
    // Replaying the same arithmetic on one thread gives the reference.
    int expected = 1;
    for (int i = 0; i < 1000; ++i) {
        expected += 1;
        expected *= 2;
        expected -= 1;
    }
    EXPECT_EQ(value, expected);
}

TEST(OwnerThread, ObjectsBuiltAndDestroyedOnTheOwnerThread) {
    // How a module holds a thread-affine component.
    struct Affine {
        Affine() : home(std::this_thread::get_id()) {}
        ~Affine() {
            EXPECT_EQ(home, std::this_thread::get_id());
        }
        bool update() const {
            return home == std::this_thread::get_id();
        }
        std::thread::id home;
    };
    OwnerThread owner;
    std::unique_ptr<Affine> affine;
    owner.run([&] { affine = std::make_unique<Affine>(); });
    for (int i = 0; i < 10; ++i)
        EXPECT_TRUE(owner.run([&] { return affine->update(); }));
    owner.run([&] { affine.reset(); });
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
