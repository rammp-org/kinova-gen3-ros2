"""Ownership and the e-stop.

The e-stop checks run FIRST in the runner's ordering, deliberately: nothing that
commands motion should be exercised on an arm whose stop path has not been proven in
this session.
"""

from harness import FAIL, PASS, REGISTRY, ZERO_TOKEN, Result

SEC = "arbitration"


@REGISTRY.add(SEC, "engaging /estop latches, and clears ownership with it")
def check_estop_engage(ctx):
    ctx.acquire("estop-probe")  # so we can see ownership being destroyed
    ctx.estop(True, reason="conformance: verifying the stop path")
    cs = ctx.control_status()
    if cs is None:
        return Result("", FAIL, "no /control_status after e-stop")
    if not cs.estopped:
        return Result("", FAIL, "estopped is still false after /estop ENGAGED")
    if cs.owned:
        return Result("", FAIL, "ownership survived the e-stop; estop() must clear it")
    return Result("", PASS, "estopped=true, ownership destroyed")


@REGISTRY.add(SEC, "the engaging source's NOT_ENGAGED releases, leaving NO owner")
def check_estop_clear(ctx):
    ctx.estop(True, reason="conformance")
    ctx.estop(False, reason="conformance")
    cs = ctx.control_status()
    if cs.estopped:
        return Result("", FAIL, "still latched after the engaging source's NOT_ENGAGED")
    if cs.owned:
        return Result(
            "", FAIL, "release returned to an OWNED state; core exits to no-owner"
        )
    return Result("", PASS, "estopped=false, unowned as designed")


@REGISTRY.add(SEC, "another source's heartbeat does not release the stop")
def check_estop_heartbeat_does_not_release(ctx):
    ctx.estop(True, reason="conformance")
    ctx.estop(False, source="conformance-other", reason="conformance: heartbeat")
    cs = ctx.control_status()
    ctx.estop(False, reason="conformance cleanup")
    if not cs.estopped:
        return Result(
            "",
            FAIL,
            "a heartbeat from a different source released the stop; any node "
            "could end another node's e-stop",
        )
    return Result("", PASS, "heartbeat ignored, latch held")


@REGISTRY.add(SEC, "a STALE engage is still honoured")
def check_estop_stale_engage(ctx):
    ctx.estop(False, reason="conformance")
    old = ctx.n.get_clock().now().to_msg()
    old.sec -= 30
    ctx.estop(True, reason="conformance: replayed", stamp=old)
    cs = ctx.control_status()
    ctx.estop(False, reason="conformance cleanup")
    if not cs.estopped:
        return Result(
            "",
            FAIL,
            "a stale STOP was refused; an engage must never be age-checked",
        )
    return Result("", PASS, "stale engage honoured")


@REGISTRY.add(SEC, "a latched MIB's clear attempts hold the stop until it releases")
def check_estop_mib_clear_attempt(ctx):
    mib, web = "conformance-mib", "conformance-web"
    ctx.estop(True, source=web, reason="conformance: button")
    ctx.estop(True, source=mib, reason="conformance: latched")
    ctx.estop(False, source=mib, clear_target=web, reason="conformance: clearing")
    during_attempt = ctx.control_status().estopped
    ctx.estop(False, source=web, reason="conformance: accepts the clear")
    after_target = ctx.control_status().estopped
    ctx.estop(False, source=mib, reason="conformance: released")
    after_mib = ctx.control_status().estopped
    if not during_attempt:
        return Result("", FAIL, "a clear attempt released the stop by itself")
    if not after_target:
        return Result(
            "",
            FAIL,
            "released before the MIB did: a clear attempt counted as the MIB "
            "going NOT_ENGAGED",
        )
    if after_mib:
        return Result("", FAIL, "still latched after every source is NOT_ENGAGED")
    return Result("", PASS, "held through the attempt, released with the MIB")


@REGISTRY.add(SEC, "acquire mints a token and takes ownership")
def check_acquire(ctx):
    r = ctx.acquire("conformance-a")
    if not r.accepted:
        return Result("", FAIL, f"acquire refused: {r.message}")
    if list(r.token) == ZERO_TOKEN:
        return Result("", FAIL, "accepted but the token is all zeros")
    cs = ctx.control_status()
    if not cs.owned or cs.owner_id != "conformance-a":
        return Result("", FAIL, f"status says owned={cs.owned} owner='{cs.owner_id}'")
    return Result("", PASS, f"owner='{cs.owner_id}' generation={r.generation}")


@REGISTRY.add(SEC, "acquiring again SEIZES: generation bumps, old token dies")
def check_seizure(ctx):
    first = ctx.acquire("conformance-incumbent")
    second = ctx.acquire("conformance-usurper")
    if not second.accepted:
        return Result("", FAIL, "second acquire was refused; grant() is meant to seize")
    if second.generation <= first.generation:
        return Result(
            "",
            FAIL,
            f"generation did not advance "
            f"({first.generation} -> {second.generation})",
        )
    if list(first.token) == list(second.token):
        return Result("", FAIL, "the same token was reissued to a new owner")
    cs = ctx.control_status()
    if cs.owner_id != "conformance-usurper":
        return Result("", FAIL, f"owner is '{cs.owner_id}' after the seizure")
    return Result(
        "",
        PASS,
        f"generation {first.generation} -> {second.generation}, "
        "incumbent dispossessed",
    )


@REGISTRY.add(SEC, "release with the WRONG token is refused")
def check_release_wrong_token(ctx):
    ctx.acquire("conformance-owner")
    bogus = [0xAB] + [0] * 15
    r = ctx.release(bogus)
    if r.released:
        return Result("", FAIL, "a stranger's token released someone else's arm")
    cs = ctx.control_status()
    if not cs.owned:
        return Result("", FAIL, "refused but ownership was dropped anyway")
    return Result("", PASS, f"refused: {r.message}")


@REGISTRY.add(SEC, "release with the right token gives the arm back")
def check_release(ctx):
    a = ctx.acquire("conformance-owner")
    r = ctx.release(a.token)
    if not r.released:
        return Result("", FAIL, f"release refused with the minted token: {r.message}")
    cs = ctx.control_status()
    if cs.owned:
        return Result("", FAIL, "still owned after a successful release")
    return Result("", PASS, "released, arm unowned")


@REGISTRY.add(SEC, "operator revoke needs no token")
def check_revoke(ctx):
    ctx.acquire("conformance-hung-client")
    r = ctx.revoke("conformance: simulating a crashed owner")
    if not r.revoked:
        return Result("", FAIL, f"revoke refused: {r.message}")
    cs = ctx.control_status()
    if cs.owned:
        return Result("", FAIL, "still owned after revoke")
    return Result("", PASS, "revoked without a token, as the recovery path requires")
