package dev.slimevr.unit

import dev.slimevr.tracking.processor.skeleton.ContactAwareHipStabilizer
import dev.slimevr.tracking.processor.skeleton.ContactAwareHipStabilizer.Contact
import dev.slimevr.tracking.processor.skeleton.ContactAwareHipStabilizer.Frame
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import org.junit.jupiter.api.Test
import kotlin.math.*
import kotlin.test.assertEquals
import kotlin.test.assertTrue

class ContactAwareHipStabilizerTests {

	private fun supine(): Frame {
		val rotation = Quaternion.fromRotationVector(Vector3(PI.toFloat() / 2f, 0f, 0f))
		return Frame(
			hip = Vector3(0f, 0.18f, 0f),
			chest = Vector3(0f, 0.20f, -0.4f),
			leftKnee = Vector3(-0.12f, 0.12f, 0.4f),
			rightKnee = Vector3(0.12f, 0.12f, 0.4f),
			head = Vector3(0f, 0.22f, -0.6f),
			chestRotation = rotation,
			hipRotation = rotation,
			floor = 0f,
		)
	}

	private fun kneeling(): Frame {
		val rotation = Quaternion.fromRotationVector(Vector3(-PI.toFloat() / 2f, 0f, 0f))
		return supine().copy(
			hip = Vector3(0f, 0.50f, 0f),
			chest = Vector3(0f, 0.48f, -0.4f),
			leftKnee = Vector3(-0.12f, 0.08f, 0f),
			rightKnee = Vector3(0.12f, 0.08f, 0f),
			head = Vector3(0f, 0.55f, -0.6f),
			chestRotation = rotation,
			hipRotation = rotation,
			outputHipY = 0.50f,
		)
	}

	private fun settle(estimator: ContactAwareHipStabilizer, frame: Frame, dt: Float = 0.01f) {
		repeat((0.7f / dt).toInt()) { estimator.update(frame, dt) }
	}

	@Test
	fun disabledIsExactPassThroughAndClearsContactImmediately() {
		val estimator = ContactAwareHipStabilizer()
		settle(estimator, supine())
		assertEquals(Contact.NONE, estimator.contact)
		assertEquals(0f, estimator.correction)
		estimator.enabled = true
		settle(estimator, supine())
		settle(estimator, supine().copy(outputHipY = 0.20f))
		assertTrue(abs(estimator.correction) > 0.01f)
		estimator.enabled = false
		assertEquals(0f, estimator.correction)
		assertEquals(0f, estimator.update(supine(), 0.01f))
		assertEquals(Contact.NONE, estimator.contact)
	}

	@Test
	fun supineWobbleIsReducedWithoutDependingOnFrameRate() {
		for (hz in listOf(60, 90, 144, 1000)) {
			val estimator = ContactAwareHipStabilizer().apply { enabled = true }
			val base = supine()
			val dt = 1f / hz
			settle(estimator, base, dt)
			assertEquals(Contact.SUPINE, estimator.contact)
			var rawEnergy = 0f
			var correctedEnergy = 0f
			repeat(hz * 3) { i ->
				val noise = 0.015f * sin(2f * PI.toFloat() * 1.5f * i * dt)
				val frame = base.copy(hip = base.hip + Vector3(0f, noise, 0f), outputHipY = base.hip.y + noise)
				val offset = estimator.update(frame, dt)
				rawEnergy += noise * noise
				correctedEnergy += (noise + offset).pow(2)
				assertTrue(abs(offset) <= 0.04f)
			}
			assertTrue(correctedEnergy < rawEnergy * 0.5f, "Insufficient suppression at $hz Hz")
		}
	}

	@Test
	fun kneelingRejectsCommonTranslationAndPreservesHipMotionRelativeToKnees() {
		val estimator = ContactAwareHipStabilizer().apply { enabled = true }
		val base = kneeling()
		settle(estimator, base)
		assertEquals(Contact.KNEELING, estimator.contact)
		val shift = Vector3(0f, 0.02f, 0f)
		val bounced = base.copy(
			hip = base.hip + shift,
			chest = base.chest + shift,
			leftKnee = base.leftKnee + shift,
			rightKnee = base.rightKnee + shift,
			outputHipY = base.hip.y + shift.y,
		)
		settle(estimator, bounced)
		assertEquals(-0.02f, estimator.correction, 0.001f)
		estimator.reset()
		settle(estimator, base)
		settle(estimator, base.copy(hip = base.hip + shift, outputHipY = base.hip.y + shift.y))
		assertEquals(0f, estimator.correction, 0.001f)
	}

	@Test
	fun standingBedsAndMissingKneeSupportDoNotEngage() {
		val estimator = ContactAwareHipStabilizer().apply { enabled = true }
		for (frame in listOf(
			supine().copy(chestRotation = Quaternion.IDENTITY),
			supine().copy(hip = Vector3(0f, 0.65f, 0f), chest = Vector3(0f, 0.65f, -0.4f)),
			kneeling().copy(leftKnee = Vector3(0f, 0.3f, 0f)),
			kneeling().copy(leftHand = Vector3(0f, 0.5f, 0f)),
		)) {
			estimator.reset()
			settle(estimator, frame)
			assertEquals(Contact.NONE, estimator.contact)
			assertEquals(0f, estimator.correction)
		}
	}

	@Test
	fun movementPoseChangesAndTrackingDiscontinuitiesReleaseContact() {
		for (next in listOf(
			supine().copy(chestRotation = Quaternion.IDENTITY),
			supine().copy(hip = Vector3(0f, 0.35f, 0f)),
			supine().copy(hipRotation = Quaternion.IDENTITY),
			supine().copy(head = Vector3(1f, 0.22f, -0.6f)),
			supine().copy(floor = 1f),
		)) {
			val estimator = ContactAwareHipStabilizer().apply { enabled = true }
			settle(estimator, supine())
			estimator.update(next, 0.01f)
			assertEquals(Contact.NONE, estimator.contact)
		}
	}

	@Test
	fun invalidInputAndLongFrameGapsClearState() {
		val estimator = ContactAwareHipStabilizer().apply { enabled = true }
		settle(estimator, supine())
		assertEquals(0f, estimator.update(supine().copy(outputHipY = Float.NaN), 0.01f))
		assertEquals(Contact.NONE, estimator.contact)
		settle(estimator, supine())
		assertEquals(0f, estimator.update(supine(), 0.3f))
		assertEquals(Contact.NONE, estimator.contact)
	}

	@Test
	fun correctionIsBoundedAndFadesOutWhenLeavingContact() {
		val estimator = ContactAwareHipStabilizer().apply { enabled = true }
		settle(estimator, supine())
		settle(estimator, supine().copy(outputHipY = 0.5f))
		assertEquals(-0.04f, estimator.correction, 0.001f)
		val standing = supine().copy(chestRotation = Quaternion.IDENTITY)
		val first = estimator.update(standing, 0.01f)
		assertTrue(first > -0.04f && first < 0f)
		settle(estimator, standing)
		assertEquals(0f, estimator.correction, 0.0001f)
	}
}
