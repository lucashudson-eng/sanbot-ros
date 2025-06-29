plugins {
    id("com.android.application")
}

android {
    compileSdk = 33

    defaultConfig {
        applicationId = "com.grin.sanbotmqtt"
        minSdk = 21
        targetSdk = 33
        versionCode = 1
        versionName = "1.0"
        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"
    }

    buildTypes {
        getByName("release") {
            isMinifyEnabled = false
            proguardFiles(
                getDefaultProguardFile("proguard-android.txt"),
                "proguard-rules.pro"
            )
        }
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }
}

repositories {
    flatDir {
        dirs("libs")
    }
}

dependencies {
    implementation(fileTree(mapOf("dir" to "libs", "include" to listOf("*.jar"))))

    implementation("androidx.appcompat:appcompat:1.6.1")
    implementation("androidx.constraintlayout:constraintlayout:2.1.4")
    implementation("androidx.annotation:annotation:1.6.0")
    testImplementation("junit:junit:4.12")
    androidTestImplementation("androidx.test:runner:1.5.2")
    androidTestImplementation("androidx.test.ext:junit:1.1.5")
    androidTestImplementation("androidx.test.espresso:espresso-core:3.5.1")
    implementation(files("libs/dsdk-Normal-release1.aar"))
    implementation("com.google.code.gson:gson:2.8.2")
    implementation("com.jakewharton:butterknife:10.2.3")
    implementation("org.eclipse.paho:org.eclipse.paho.client.mqttv3:1.2.5")
    implementation("org.nanohttpd:nanohttpd:2.3.1")
    annotationProcessor("com.jakewharton:butterknife-compiler:10.2.3")
}

configurations.all {
    exclude(group = "org.jetbrains.kotlin")
}