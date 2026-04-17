#pragma once
#include "Event.h"

struct AddToFlatEvent : Event { class Drawable* drawable; AddToFlatEvent(class Drawable* d) : drawable(d) {} };
struct AddToLitEvent  : Event { class Drawable* drawable; AddToLitEvent(class Drawable* d)  : drawable(d) {} };