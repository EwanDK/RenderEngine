#include "Drawable.h"
#include "DrawableEvents.h"
#include "EventBus.h"

void Drawable::AddToFlat(EventBus& bus) { bus.PublishImmediately(AddToFlatEvent{ this }); }
void Drawable::AddToLit(EventBus& bus)  { bus.PublishImmediately(AddToLitEvent{ this }); }