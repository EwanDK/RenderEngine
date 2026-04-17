#include "Drawable.h"
#include "DrawableEvents.h"
#include "EventBus.h"

void Drawable::AddToFlat(EventBus& bus) { EventBus::Get().PublishImmediately(AddToFlatEvent{ this }); }
void Drawable::AddToLit(EventBus& bus)  { EventBus::Get().PublishImmediately(AddToLitEvent{ this }); }