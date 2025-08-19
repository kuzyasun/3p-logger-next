#include "observer.h"

#include "freertos/FreeRTOS.h"
#include "stddef.h"

// Observer implementation
Observer *ObserverCreate(const char *name, void *context, observer_update_fn update_fn, void *user_data) {
    Observer *observer = (Observer *)malloc(sizeof(Observer));
    if (!observer) {
        return NULL;
    }

    observer->name = name;
    observer->context = context;
    observer->update = update_fn;
    observer->cleanup = NULL;  // Default to no cleanup
    observer->user_data = user_data;

    return observer;
}

void ObserverDelete(Observer *observer) {
    if (observer) {
        // Call custom cleanup if provided
        if (observer->cleanup) {
            observer->cleanup(observer);
        }
        free(observer);
    }
}

// Subject implementation
Subject *SubjectCreate(void) {
    Subject *subject = (Subject *)malloc(sizeof(Subject));
    if (!subject) {
        return NULL;
    }

    // Initialize with NULL functions - must be set by derived classes
    subject->attach = NULL;
    subject->detach = NULL;
    subject->notify = NULL;
    subject->cleanup = NULL;  // Will be set by derived classes

    return subject;
}

void SubjectCleanup(void *subject) {
    if (subject) {
        free(subject);
    }
}

// Notifier implementation
static void NotifierAttach(void *subject, Observer *observer) {
    Notifier *notifier = (Notifier *)subject;
    if (notifier->observers && observer) {
        notifier->observers->Add(notifier->observers, observer);
    }
}

static void NotifierDetach(void *subject, Observer *observer) {
    Notifier *notifier = (Notifier *)subject;
    if (notifier->observers && observer) {
        notifier->observers->Remove(notifier->observers, observer);
    }
}

static void NotifierNotify(void *subject, void *data) {
    Notifier *notifier = (Notifier *)subject;
    if (!notifier->observers) {
        return;
    }

    for (int i = 0; i < notifier->observers->Total; i++) {
        Observer *observer = (Observer *)(notifier->observers->pListPointArray[i]);
        if (observer && observer->update) {
            observer->update(notifier, observer, data);
        }
    }
}

static void NotifierCleanupInternal(void *subject) {
    Notifier *notifier = (Notifier *)subject;
    if (notifier) {
        if (notifier->observers) {
            notifier->observers->Delete(notifier->observers);
        }
        free(notifier);
    }
}

Notifier *NotifierCreate(void *owner) {
    Notifier *notifier = (Notifier *)malloc(sizeof(Notifier));
    if (!notifier) {
        return NULL;
    }

    // Initialize the subject interface
    notifier->subject.attach = NotifierAttach;
    notifier->subject.detach = NotifierDetach;
    notifier->subject.notify = NotifierNotify;
    notifier->subject.cleanup = NotifierCleanupInternal;

    // Initialize notifier-specific fields
    notifier->observers = ListCreate();
    notifier->owner = owner;

    return notifier;
}

void NotifierDelete(Notifier *notifier) {
    if (notifier) {
        notifier->subject.cleanup(notifier);
    }
}