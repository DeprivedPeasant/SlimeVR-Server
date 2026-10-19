package dev.slimevr.tracking.processor.skeleton

import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import kotlin.math.*

/**
 * Experimental vertical-only output correction. Contacts are hypotheses, not measurements.
 * Always feed uncorrected skeleton positions back into this estimator, never its own output.
 */
class ContactAwareHipStabilizer {

	enum class Contact { NONE, SUPINE, KNEELING }

	data class Frame(
		val hip: Vector3,
		val chest: Vector3,
		val leftKnee: Vector3,
		val rightKnee: Vector3,
		val head: Vector3,
		val chestRotation: Quaternion,
		val hipRotation: Quaternion,
		val floor: Float,
		val outputHipY: Float = hip.y,
		val leftHand: Vector3? = null,
		val rightHand: Vector3? = null,
	)

	var enabled = false
		set(value) {
			if (field != value) reset()
			field = value
		}
	var contact = Contact.NONE
		private set
	var correction = 0f
		private set
	private var candidate = Contact.NONE
	private var candidateTime = 0f
	private var anchor: Frame? = null
	private var previous: Frame? = null
	private var velocity = Vector3.NULL
	private var movingTime = 0f

	fun reset() {
		contact = Contact.NONE
		candidate = Contact.NONE
		candidateTime = 0f
		anchor = null
		previous = null
		velocity = Vector3.NULL
		movingTime = 0f
		correction = 0f
	}

	/** Returns a Y offset in metres, capped at 4 cm. A disabled estimator returns exactly zero. */
	fun update(frame: Frame, dt: Float): Float {
		if (!enabled || !valid(frame) || !dt.isFinite() || dt <= 0f || dt > 0.1f) {
			reset()
			return 0f
		}
		val prev = previous
		previous = frame
		if (prev == null) return 0f
		// A tracking-origin discontinuity must not leave a contact in the old space.
		if ((frame.head - prev.head).len() > 0.15f || abs(frame.floor - prev.floor) > 0.005f) {
			reset()
			return 0f
		}
		val alpha = 1f - exp(-dt / 0.12f)
		velocity += ((frame.hip - prev.hip) / dt - velocity) * alpha
		movingTime = if (velocity.len() > 0.3f) movingTime + dt else 0f
		val moving = movingTime > 0.08f
		val pose = detect(frame, contact != Contact.NONE)
		var reference = anchor
		if (contact != Contact.NONE && reference != null) {
			val changed = (frame.hip - reference.hip).len() > 0.10f ||
				(contact == Contact.SUPINE && abs(frame.hip.y - reference.hip.y) > 0.045f) ||
				(contact == Contact.KNEELING && abs(kneeY(frame) - kneeY(reference)) > 0.06f) ||
				frame.hipRotation.angleToR(reference.hipRotation) > 0.20f ||
				frame.chestRotation.angleToR(reference.chestRotation) > 0.30f
			if (pose != contact || moving || changed) {
				contact = Contact.NONE
				anchor = null
				candidate = Contact.NONE
				candidateTime = 0f
			}
		}
		if (contact == Contact.NONE) {
			if (pose != Contact.NONE && !moving) {
				if (candidate != pose) {
					candidate = pose
					candidateTime = 0f
					anchor = frame
				}
				val start = anchor!!
				// Require a settled pose, not just a slow passage through one.
				if ((frame.hip - start.hip).len() > 0.025f ||
					frame.hipRotation.angleToR(start.hipRotation) > 0.10f
				) {
					anchor = frame
					candidateTime = 0f
				}
				candidateTime += dt
				if (candidateTime >= 0.4f) {
					contact = pose
					anchor = frame
				}
			} else {
				candidate = Contact.NONE
				candidateTime = 0f
				anchor = null
			}
		}
		reference = anchor
		val desired = if (contact != Contact.NONE && reference != null) {
			val targetY = if (contact == Contact.SUPINE) {
				reference.hip.y
			} else {
				// Preserve deliberate pelvis movement relative to the knees, while rejecting
				// common vertical translation caused by the head-to-spine reconstruction.
				frame.hip.y - (kneeY(frame) - kneeY(reference))
			}
			(targetY - frame.outputHipY).coerceIn(-0.04f, 0.04f)
		} else {
			0f
		}
		correction += (desired - correction) * (1f - exp(-dt / 0.08f))
		return correction
	}

	private fun detect(frame: Frame, retaining: Boolean): Contact {
		val margin = if (retaining) 0.04f else 0f
		val chestHeight = frame.chest.y - frame.floor
		val hipHeight = frame.hip.y - frame.floor
		val horizontal = abs(frame.chestRotation.sandwichUnitY().y) < 0.5f + margin
		val facingY = -frame.chestRotation.sandwichUnitZ().y
		if (!horizontal) return Contact.NONE
		if (facingY > 0.65f - margin &&
			abs(frame.hipRotation.sandwichUnitY().y) < 0.55f + margin &&
			hipHeight in (0.03f - margin)..(0.32f + margin) &&
			chestHeight in (0.02f - margin)..(0.40f + margin)
		) {
			return Contact.SUPINE
		}
		val kneesNearFloor = listOf(frame.leftKnee, frame.rightKnee).all {
			it.y - frame.floor in -0.08f..(0.16f + margin)
		}
		val handsNearFloor = listOfNotNull(frame.leftHand, frame.rightHand).all {
			it.y - frame.floor in -0.08f..(0.25f + margin)
		}
		if (facingY < -0.55f + margin &&
			kneesNearFloor &&
			handsNearFloor &&
			hipHeight in 0.20f..0.95f &&
			chestHeight in 0.20f..0.95f &&
			frame.hip.y - kneeY(frame) in 0.15f..0.65f
		) {
			return Contact.KNEELING
		}
		return Contact.NONE
	}

	private fun kneeY(frame: Frame): Float = (frame.leftKnee.y + frame.rightKnee.y) / 2f

	private fun valid(frame: Frame): Boolean = listOfNotNull(
		frame.hip,
		frame.chest,
		frame.leftKnee,
		frame.rightKnee,
		frame.head,
		frame.leftHand,
		frame.rightHand,
	).all { it.x.isFinite() && it.y.isFinite() && it.z.isFinite() } &&
		frame.floor.isFinite() &&
		frame.outputHipY.isFinite() &&
		listOf(frame.chestRotation, frame.hipRotation).all {
			it.w.isFinite() &&
				it.x.isFinite() &&
				it.y.isFinite() &&
				it.z.isFinite() &&
				abs(it.dot(it) - 1f) < 0.1f
		}
}
