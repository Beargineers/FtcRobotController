package org.beargineers.beta

import com.qualcomm.robotcore.hardware.LED
import org.beargineers.platform.Hardware
import kotlin.time.Duration

class LedIndicator (robot: BetaRobot): Hardware(robot) {
    object LedMods{
        var ledRedBlinkingStopAt: Long = 0L
        var ledRedBlinkingInterval: Long = 0L
        var ledContinuousRedBlinking: Boolean = false
        var ledCounter: Int = 0
    }
    inner class Led(name: String) {
        val red = getHardware<LED>(name+"red")
        val green = getHardware<LED>(name+"green")
        fun green(){
            red.off()
            green.on()
        }

        fun red(){
            red.on()
            green.off()
        }

        fun off(){
            red.off()
            green.off()
        }

        fun yellow(){
            red.on()
            green.on()
        }
    }

    fun blinkRed(interval: Duration, duration: Duration) {
        LedMods.ledRedBlinkingStopAt = System.currentTimeMillis() + duration.inWholeMilliseconds
        LedMods.ledRedBlinkingInterval = interval.inWholeMilliseconds
    }

    fun blinkRed(blinkCount: Int, duration: Duration) {
        LedMods.ledRedBlinkingStopAt = System.currentTimeMillis() + duration.inWholeMilliseconds
        LedMods.ledRedBlinkingInterval = duration.inWholeMilliseconds / blinkCount
    }

    fun continuousRedBlinkingON(interval: Duration) {
        LedMods.ledContinuousRedBlinking = true
        LedMods.ledRedBlinkingInterval = interval.inWholeMilliseconds
        LedMods.ledRedBlinkingStopAt = 0L
    }

    fun continuousRedBlinkingOFF() {
        LedMods.ledContinuousRedBlinking = false
    }

    fun setCounter(count: Int){
        LedMods.ledCounter = count
    }

    fun showCounter(){
        when(LedMods.ledCounter){
            0 -> {
                led1.off()
                led2.off()
                led3.off()
            }
            1 -> {
                led1.green()
                led2.off()
                led3.off()
            }
            2 -> {
                led1.green()
                led2.green()
                led3.off()
            }
            3 -> {
                led1.green()
                led2.green()
                led3.green()
            }

        }
    }


    val led1 = Led("led1")
    val led2 = Led("led2")
    val led3 = Led("led3")

    override fun init() {
        led1.off()
        led2.off()
        led3.off()
    }

    override fun loop() {
        val now = System.currentTimeMillis()
        val timeLeftToBlink = LedMods.ledRedBlinkingStopAt - now
        if (LedMods.ledRedBlinkingStopAt != 0L) {
            if (System.currentTimeMillis() >= LedMods.ledRedBlinkingStopAt) {
                LedMods.ledRedBlinkingStopAt = 0L
            } else if (timeLeftToBlink % LedMods.ledRedBlinkingInterval < LedMods.ledRedBlinkingInterval / 2) {
                led1.red()
                led2.red()
                led3.red()
            } else {
                showCounter()
            }

        }else if(LedMods.ledContinuousRedBlinking){
                if (now % LedMods.ledRedBlinkingInterval < LedMods.ledRedBlinkingInterval / 2){
                    led1.red()
                    led2.red()
                    led3.red()
                }else{
                    showCounter()
                }
        }else {
            showCounter()
        }
    }

    override fun stop() {
        led1.red()
        led2.red()
        led3.red()
    }
}