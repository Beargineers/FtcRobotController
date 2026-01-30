package org.beargineers.beta

import com.qualcomm.robotcore.hardware.LED
import org.beargineers.platform.Hardware
import kotlin.time.Duration

class LedIndicator (robot: BetaRobot): Hardware(robot) {
    object ledMods{
        var ledRedBlinkingStopAt: Long = 0L
        var ledRedBlinkingInterval: Long = 0L
        var ledCounter: Int = 0
    }
    val led1red: LED by hardware("led1red")
    val led1green: LED by hardware("led1green")
    val led2red: LED by hardware("led2red")
    val led2green: LED by hardware("led2green")
    val led3red: LED by hardware("led3red")
    val led3green: LED by hardware("led3green")

    class Led(val red: LED, val green: LED) {
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
        ledMods.ledRedBlinkingStopAt = System.currentTimeMillis() + duration.inWholeMilliseconds
        ledMods.ledRedBlinkingInterval = interval.inWholeMilliseconds
    }

    fun blinkRed(blinkCount: Int, duration: Duration) {
        ledMods.ledRedBlinkingStopAt = System.currentTimeMillis() + duration.inWholeMilliseconds
        ledMods.ledRedBlinkingInterval = duration.inWholeMilliseconds / blinkCount
    }

    fun setCounter(count: Int){
        ledMods.ledCounter = count
    }

    fun showCounter(){
        when(ledMods.ledCounter){
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


    val led1 = Led(led1red, led1green)
    val led2 = Led(led2red, led2green)
    val led3 = Led(led3red, led3green)

    override fun init() {
        led1.off()
        led2.off()
        led3.off()
    }

    override fun loop() {
        val now = System.currentTimeMillis()
        val timeLeftToBlink = ledMods.ledRedBlinkingStopAt - now
        if (ledMods.ledRedBlinkingStopAt != 0L) {
            if (System.currentTimeMillis() >= ledMods.ledRedBlinkingStopAt) {
                ledMods.ledRedBlinkingStopAt = 0L
            }else if(timeLeftToBlink % ledMods.ledRedBlinkingInterval < ledMods.ledRedBlinkingInterval/2){
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