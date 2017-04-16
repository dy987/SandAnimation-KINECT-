#include "kinect.h"

void ProcessDepth(INuiSensor* m_pNuiSensor, HANDLE m_pDepthStreamHandle, USHORT *m_depth)
{
	HRESULT hr;
	NUI_IMAGE_FRAME imageFrame;

	// Attempt to get the depth frame
	hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_pDepthStreamHandle, 0, &imageFrame);
	if(FAILED(hr)) {
		return;
	}

	BOOL nearMode;
	INuiFrameTexture* pTexture;

	// Get the depth image pixel texture
	hr = m_pNuiSensor->NuiImageFrameGetDepthImagePixelFrameTexture(
		m_pDepthStreamHandle, &imageFrame, &nearMode, &pTexture);
	if(FAILED(hr)) {
		goto ReleaseFrame;
	}

	NUI_LOCKED_RECT LockedRect;

	// Lock the frame data so the Kinect knows not to modify it while we're reading it
	pTexture->LockRect(0, &LockedRect, NULL, 0);

	// Make sure we've received valid data
	if(LockedRect.Pitch != 0) {
		// Get the min and max reliable depth for the current frame
		int minDepth = (nearMode ? NUI_IMAGE_DEPTH_MINIMUM_NEAR_MODE : NUI_IMAGE_DEPTH_MINIMUM) >> NUI_IMAGE_PLAYER_INDEX_SHIFT;
		int maxDepth = (nearMode ? NUI_IMAGE_DEPTH_MAXIMUM_NEAR_MODE : NUI_IMAGE_DEPTH_MAXIMUM) >> NUI_IMAGE_PLAYER_INDEX_SHIFT;

		USHORT * rgbrun = m_depth;
		const NUI_DEPTH_IMAGE_PIXEL * pBufferRun = reinterpret_cast<const NUI_DEPTH_IMAGE_PIXEL *>(LockedRect.pBits);

		for(int i = 0; i < HEIGHT; i++) {
			for(int j = 0; j < WIDTH; j++) {
				// discard the portion of the depth that contains only the player index
				USHORT depth = pBufferRun[i * WIDTH + j].depth;

				// To convert to a byte, we're discarding the most-significant
				// rather than least-significant bits.
				// We're preserving detail, although the intensity will "wrap."
				// Values outside the reliable depth range are mapped to 0 (black).

				// Note: Using conditionals in this loop could degrade performance.
				// Consider using a lookup table instead when writing production code.
				//BYTE intensity = static_cast<BYTE>(depth >= minDepth && depth <= maxDepth ? (depth << 8) / maxDepth : WHITE);

				// Write out 1channels img byte
				//m_depth[i * WIDTH + j] = ~intensity;
				m_depth[i * WIDTH + j] = depth;
			}
		}
	}

	// We're done with the texture so unlock it
	pTexture->UnlockRect(0);

	pTexture->Release();

ReleaseFrame:
	// Release the frame
	m_pNuiSensor->NuiImageStreamReleaseFrame(m_pDepthStreamHandle, &imageFrame);
}

void ProcessColor(INuiSensor* m_pNuiSensor, HANDLE m_pColorStreamHandle, UCHAR *m_colorRGB)
{
	HRESULT hr;
    NUI_IMAGE_FRAME imageFrame;

    // Attempt to get the color frame
    hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_pColorStreamHandle, 0, &imageFrame);
    if(FAILED(hr)) {
        return;
    }

    INuiFrameTexture * pTexture = imageFrame.pFrameTexture;
    NUI_LOCKED_RECT LockedRect;

    // Lock the frame data so the Kinect knows not to modify it while we're reading it
    pTexture->LockRect(0, &LockedRect, NULL, 0);

    // Make sure we've received valid data
    if(LockedRect.Pitch != 0) {
        // Draw the data with Direct2D
        BYTE *m_colorRGBX = static_cast<BYTE *>(LockedRect.pBits);

		const int R = 2, G = 1, B = 0;
		int index1;
		for(int i = 0; i < HEIGHT; ++i) {
			index1 = i * WIDTH;
			for(int j = 0; j < WIDTH; ++j) {
				m_colorRGB[index1 * 3 + R] = m_colorRGBX[index1 * 4 + R];
				m_colorRGB[index1 * 3 + G] = m_colorRGBX[index1 * 4 + G];
				m_colorRGB[index1 * 3 + B] = m_colorRGBX[index1 * 4 + B];
				++index1;
			}
		}
	}

    // We're done with the texture so unlock it
    pTexture->UnlockRect(0);

    // Release the frame
    m_pNuiSensor->NuiImageStreamReleaseFrame(m_pColorStreamHandle, &imageFrame);
}