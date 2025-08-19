#pragma once

#include <stddef.h>
#include <stdlib.h>

#include "util/list.h"

#ifdef __cplusplus
extern "C" {
#endif

// Forward declarations
typedef struct _Observer Observer;
typedef struct _Subject Subject;
typedef struct _Notifier Notifier;

// Callback function types with proper type safety
typedef void (*observer_update_fn)(Notifier *notifier, Observer *observer, void *data);
typedef void (*observer_cleanup_fn)(Observer *observer);

// Observer structure with improved design
typedef struct _Observer {
    const char *name;             // Observer name for debugging
    void *context;                // User context data
    observer_update_fn update;    // Update callback function
    observer_cleanup_fn cleanup;  // Cleanup function (optional)
    void *user_data;              // Additional user data
} Observer;

// Subject interface (what observers observe)
typedef struct _Subject {
    void (*attach)(void *subject, Observer *observer);
    void (*detach)(void *subject, Observer *observer);
    void (*notify)(void *subject, void *data);
    void (*cleanup)(void *subject);
} Subject;

// Notifier implementation of Subject
typedef struct _Notifier {
    Subject subject;  // Subject interface
    List *observers;  // List of attached observers
    void *owner;      // Pointer to the owning object (e.g., wifi_t)
} Notifier;

// Creation functions with improved safety
Observer *ObserverCreate(const char *name, void *context, observer_update_fn update_fn, void *user_data);
void ObserverDelete(Observer *observer);

Subject *SubjectCreate(void);
void SubjectCleanup(void *subject);

Notifier *NotifierCreate(void *owner);
void NotifierDelete(Notifier *notifier);

// Convenience macros for type-safe observer creation
#define OBSERVER_CREATE(name, context, update_fn) ObserverCreate(name, context, update_fn, NULL)

#define OBSERVER_CREATE_WITH_DATA(name, context, update_fn, user_data) ObserverCreate(name, context, update_fn, user_data)

// Type-safe notifier access macro
#define NOTIFIER_OWNER(notifier, type) ((type *)((notifier)->owner))

#ifdef __cplusplus
}
#endif