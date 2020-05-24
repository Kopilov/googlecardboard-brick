package com.github.kopilov.cardboard

import android.content.Context
import android.content.res.AssetManager
import android.util.Log
import java.io.File
import java.io.FileOutputStream
import java.io.IOException
import java.lang.Exception

class Util () {
    fun copyAssets(context: Context, filesToCopy: Iterable<String>) {
        val assetManager: AssetManager = context.assets
        for (filename in filesToCopy) {
            try {
                assetManager.open(filename).use { input ->
                    val outFile = File(context.getExternalFilesDir(null), filename)
                    FileOutputStream(outFile).use { output ->
                        input.copyTo(output)
                    }
                }
            } catch (e: IOException) {
                Log.e("tag", "Failed to copy asset file: $filename", e)
            }
        }
    }

}
