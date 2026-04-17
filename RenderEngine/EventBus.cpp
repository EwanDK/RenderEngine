#include "EventBus.h"
#include <algorithm>

void EventBus::Unsubscribe(SubscriptionID id) {
    for (auto& [type, list] : subscribers) {
        auto it = std::remove_if(list.begin(), list.end(),
            [id](const Subscriber& s) { return s.id == id; });
        if (it != list.end()) {
            list.erase(it, list.end());
            return;
        }
    }
}

void EventBus::Flush(float currentTime) {
    // --- Pending queue: capped at MaxEventsPerFrame, leftovers carry to next frame ---

    // Swap so Publish() calls during dispatch land in a fresh pendingQueue
    std::swap(pendingQueue, flushBuffer);

    int toProcess = std::min(static_cast<int>(flushBuffer.size()), MaxEventsPerFrame);
    for (int i = 0; i < toProcess; i++)
        Dispatch(flushBuffer[i].type, *flushBuffer[i].data);

    // Leftovers go to the front of next frame's queue, before any events published during this flush
    if (toProcess < static_cast<int>(flushBuffer.size())) {
        std::vector<PendingEvent> next;
        next.reserve(flushBuffer.size() - toProcess + pendingQueue.size());
        next.insert(next.end(),
            std::make_move_iterator(flushBuffer.begin() + toProcess),
            std::make_move_iterator(flushBuffer.end()));
        next.insert(next.end(),
            std::make_move_iterator(pendingQueue.begin()),
            std::make_move_iterator(pendingQueue.end()));
        pendingQueue = std::move(next);
    }
    flushBuffer.clear();

    // --- Delayed queue: fully resolves all elapsed events, independent of pending cap ---

    std::vector<DelayedEvent> toFire;
    auto it = delayedQueue.begin();
    while (it != delayedQueue.end()) {
        if (it->resolveAt <= currentTime) {
            toFire.push_back(std::move(*it));
            it = delayedQueue.erase(it);
        } else {
            ++it;
        }
    }

    std::sort(toFire.begin(), toFire.end(),
        [](const DelayedEvent& a, const DelayedEvent& b) { return a.resolveAt < b.resolveAt; });
    for (auto& de : toFire)
        Dispatch(de.type, *de.data);
}

void EventBus::Dispatch(std::type_index type, const Event& event) {
    auto it = subscribers.find(type);
    if (it != subscribers.end())
        for (auto& sub : it->second)
            sub.callback(event);
}