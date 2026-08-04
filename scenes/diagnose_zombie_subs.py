# Checks Script Editor's persistent Python namespace for leftover subscription/
# node handles from earlier runs -- "File > New Stage" resets the USD scene but
# NOT this namespace, so a callback subscribed hours ago (e.g. from before the
# self-healing teardown-ordering fix existed) could still be alive and firing
# every frame, using a closure frozen on whatever V_BAT/TORQUE_MULTIPLIER/wheel
# objects existed back then -- completely unaffected by anything edited since.
# Read-only, no side effects -- safe to run any time.

import gc

suspicious_keywords = ("sub", "joy", "update", "teleop", "node")
suspicious_globals = {
    k: v for k, v in globals().items()
    if not k.startswith("__") and any(kw in k.lower() for kw in suspicious_keywords)
}
print("--- Suspicious names currently in this Script Editor session's namespace ---")
for k, v in suspicious_globals.items():
    print("%s = %r" % (k, v))
if not suspicious_globals:
    print("(none found by name -- doesn't rule out an anonymous/orphaned one, see below)")

# Broader check: count live objects of the actual subscription type, regardless
# of what name (if any) still references them -- catches a subscription whose
# handle variable was itself overwritten/lost (so no longer in globals() at all)
# but that never got garbage-collected because something else still holds a ref.
try:
    import omni.kit.app
    sub_type = type(omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(lambda e: None))
    live_subs = [obj for obj in gc.get_objects() if isinstance(obj, sub_type)]
    print("--- Live update-event subscription objects in the process: %d ---" % len(live_subs))
    print("(1 is expected -- the throwaway one this check just created to learn the type. More than 1 means at least one real leftover subscription is still active.)")
except Exception as e:
    print("Broader check failed (non-fatal):", e)
