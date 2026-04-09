#pragma once
#include "Event.h"

struct AddToFlatEvent : Event { class Drawable* drawable; };
struct AddToLitEvent  : Event { class Drawable* drawable; };