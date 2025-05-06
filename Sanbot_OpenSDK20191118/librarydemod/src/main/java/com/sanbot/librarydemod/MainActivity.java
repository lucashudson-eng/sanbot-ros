package com.sanbot.librarydemod;

import android.content.Intent;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.View;

import com.sanbot.librarydemod.video.MediaControlActivity;
import com.sanbot.librarydemod.video.SoftDecodeActivity;

public class MainActivity extends AppCompatActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

//        startActivity(new Intent(MainActivity.this,MediaControlActivity.class));
    }

    public void mainStreamHard(View view) {
        startActivity(new Intent(MainActivity.this,MediaControlActivity.class).putExtra("type",1));
    }

    public void subStreamHard(View view) {
        startActivity(new Intent(MainActivity.this,MediaControlActivity.class).putExtra("type",2));
    }

    public void subStreamNV21(View view) {
        startActivity(new Intent(MainActivity.this,SoftDecodeActivity.class).putExtra("type",2));
    }

    public void mainStreamNV21(View view) {
        startActivity(new Intent(MainActivity.this,SoftDecodeActivity.class).putExtra("type",1));
    }
}
