#pragma once
#include <cstdint>
#include <functional>
#include <memory>
#include <typeindex>
#include <unordered_map>
#include <vector>
#include "Event.h"

class EventBus {
public:
    using SubscriptionID = uint64_t;

    // --- Subscription ---

    template<typename EventT>
    SubscriptionID Subscribe(std::function<void(const EventT&)> callback) {
        SubscriptionID id = nextID++;
        subscribers[typeid(EventT)].push_back({ id,
            [callback](const Event& e) { callback(static_cast<const EventT&>(e)); }
        });
        return id;
    }

    void Unsubscribe(SubscriptionID id);

    // --- Publishing ---

    // Queued: dispatched on the next Flush()
    template<typename EventT>
    void Publish(EventT event) {
        pendingQueue.push_back({ typeid(EventT), std::make_unique<EventT>(std::move(event)) });
    }

    // Immediate: dispatched inline, bypasses the queue
    template<typename EventT>
    void PublishImmediately(const EventT& event) {
        Dispatch(typeid(EventT), event);
    }

    // Delayed: dispatched on the first Flush() where currentTime >= publishTime + delay
    template<typename EventT>
    void PublishDelayed(EventT event, float delay, float currentTime) {
        delayedQueue.push_back({ currentTime + delay, typeid(EventT),
            std::make_unique<EventT>(std::move(event)) });
    }

    // Called once per frame. Resolves pending and elapsed delayed events.
    void Flush(float currentTime);
    
    void SetMaxEventsPerFrame(int max){MaxEventsPerFrame = max;}
    int GetMaxEventsPerFrame(){return MaxEventsPerFrame;}

private:
    struct Subscriber {
        SubscriptionID id;
        std::function<void(const Event&)> callback;
    };

    struct PendingEvent {
        std::type_index type;
        std::unique_ptr<Event> data;
    };

    struct DelayedEvent {
        float resolveAt;
        std::type_index type;
        std::unique_ptr<Event> data;
    };

    void Dispatch(std::type_index type, const Event& event);

    std::unordered_map<std::type_index, std::vector<Subscriber>> subscribers;
    std::vector<PendingEvent> pendingQueue;
    std::vector<PendingEvent> flushBuffer;  // double-buffer: swapped in during Flush so re-entrant Publish() goes to next frame
    std::vector<DelayedEvent> delayedQueue;
    SubscriptionID nextID = 0;
    
    int MaxEventsPerFrame = 60;
};