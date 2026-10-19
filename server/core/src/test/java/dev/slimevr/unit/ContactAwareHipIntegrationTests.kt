package dev.slimevr.unit

import com.google.flatbuffers.FlatBufferBuilder
import dev.slimevr.tracking.processor.HumanPoseManager
import dev.slimevr.tracking.processor.config.SkeletonConfigToggles
import dev.slimevr.tracking.processor.skeleton.ContactAwareHipStabilizer.Contact
import dev.slimevr.tracking.processor.skeleton.ContactAwareHipStabilizer.Frame
import dev.slimevr.tracking.trackers.Tracker
import dev.slimevr.tracking.trackers.TrackerPosition
import dev.slimevr.tracking.trackers.TrackerStatus
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import org.junit.jupiter.api.Test
import solarxr_protocol.rpc.settings.ModelToggles
import java.nio.ByteBuffer
import kotlin.math.PI
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertTrue

class ContactAwareHipIntegrationTests {

	private fun setup(): Pair<HumanPoseManager, Tracker> {
		val trackers = listOf(
			TrackerPosition.HEAD, TrackerPosition.CHEST, TrackerPosition.HIP,
			TrackerPosition.LEFT_UPPER_LEG, TrackerPosition.RIGHT_UPPER_LEG,
			TrackerPosition.LEFT_LOWER_LEG, TrackerPosition.RIGHT_LOWER_LEG,
			TrackerPosition.LEFT_FOOT, TrackerPosition.RIGHT_FOOT,
		).mapIndexed { id, position ->
			Tracker(
				null, id, "test:$position", position.name, position,
				hasPosition = position == TrackerPosition.HEAD,
				hasRotation = true, allowReset = false, allowMounting = false,
				isHmd = position == TrackerPosition.HEAD, trackRotDirection = false,
			).apply { status = TrackerStatus.OK }
		}
		val manager = HumanPoseManager(trackers)
		trackers.first().position = Vector3(0f, manager.userHeightFromConfig, 0f)
		manager.skeleton.legTweaks.resetFloorLevel()
		manager.update()
		return manager to trackers.first()
	}

	@Test
	fun defaultOffAndEnabledStandingMatchOriginalOutputsExactly() {
		val (original) = setup()
		val (experimental) = setup()
		assertFalse(original.getToggle(SkeletonConfigToggles.CONTACT_AWARE_HIPS))
		experimental.setToggle(SkeletonConfigToggles.CONTACT_AWARE_HIPS, true)
		repeat(10) {
			original.update()
			experimental.update()
			original.computedTrackers.zip(experimental.computedTrackers).forEach { (a, b) ->
				assertEquals(a.position, b.position)
				assertEquals(a.getRotation(), b.getRotation())
			}
		}
	}

	@Test
	fun trackingLossResetAndPauseClearPreviouslyAcquiredContact() {
		val (manager, head) = setup()
		manager.setToggle(SkeletonConfigToggles.CONTACT_AWARE_HIPS, true)
		val estimator = manager.skeleton.contactAwareHips
		val rotation = Quaternion.fromRotationVector(Vector3(PI.toFloat() / 2f, 0f, 0f))
		val frame = Frame(
			Vector3(0f, 0.18f, 0f),
			Vector3(0f, 0.2f, -0.4f),
			Vector3(-0.1f, 0.1f, 0.4f),
			Vector3(0.1f, 0.1f, 0.4f),
			Vector3(0f, 0.22f, -0.6f),
			rotation,
			rotation,
			0f,
		)
		fun acquire() {
			repeat(60) { estimator.update(frame, 0.01f) }
			assertEquals(Contact.SUPINE, estimator.contact)
		}
		acquire()
		head.status = TrackerStatus.TIMED_OUT
		manager.update()
		assertEquals(Contact.NONE, estimator.contact)
		head.status = TrackerStatus.OK
		acquire()
		manager.resetTrackersYaw("test")
		assertEquals(Contact.NONE, estimator.contact)
		acquire()
		manager.skeleton.setPauseTracking(true, "test")
		manager.update()
		assertEquals(Contact.NONE, estimator.contact)
	}

	@Test
	fun settingsRoundTripDistinguishesExplicitFalseFromAnOlderMissingField() {
		for (value in listOf<Boolean?>(null, false, true)) {
			val builder = FlatBufferBuilder(128)
			// Match the GUI's settings writer: explicit false must survive serialization.
			builder.forceDefaults(true)
			ModelToggles.startModelToggles(builder)
			if (value != null) ModelToggles.addContactAwareHips(builder, value)
			builder.finish(ModelToggles.endModelToggles(builder))
			val toggles = ModelToggles.getRootAsModelToggles(ByteBuffer.wrap(builder.sizedByteArray()))
			assertEquals(value != null, toggles.hasContactAwareHips())
			if (value != null) {
				assertEquals(value, toggles.contactAwareHips())
			} else {
				assertFalse(toggles.contactAwareHips())
			}
		}
		assertTrue(SkeletonConfigToggles.CONTACT_AWARE_HIPS.configKey == "contactAwareHips")
	}
}
