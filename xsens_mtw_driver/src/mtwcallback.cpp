#include "../include/xsens/xsensdeviceapi/xsdevice.h"
#include "../include/xsens/xstypes/xsdatapacket.h"

#include "../include/xsens_mtw_driver/mtwcallback.hpp"

//----------------------------------------------------------------------
// Callback handler for MTw
// Handles onDataAvailable callbacks for MTW devices
//----------------------------------------------------------------------

MtwCallback::MtwCallback(int mtwIndex, XsDevice* device, size_t maxBufferSize)
	: m_mtwIndex(mtwIndex)
	, m_device(device)
	, m_maxBufferSize(maxBufferSize)
	, m_clock(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME))
{}

bool MtwCallback::dataAvailable() const
{
	XsMutexLocker lock(m_mutex);
	return !m_packetBuffer.empty();

	//std::unique_lock<std::mutex> lock(m_stdmutex);
	//return !m_rosBuffer.empty();
}

XsDataPacket const * MtwCallback::getOldestPacket() const
{
	XsMutexLocker lock(m_mutex);
	XsDataPacket const * packet = &m_packetBuffer.front();
	return packet;
}

rclcpp::Time MtwCallback::getOldestReceiveStamp() const
{
	XsMutexLocker lock(m_mutex);
	if (m_receiveStampBuffer.empty())
		return rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
	return m_receiveStampBuffer.front();
}

// Returns empty packet on timeout
RosXsDataPacket MtwCallback::next(const std::chrono::milliseconds &timeout)
{
	RosXsDataPacket packet;

	std::unique_lock<std::mutex> lock(m_stdmutex);
	//XsMutexLocker lock(m_mutex);

	if (m_condition.wait_for(lock, timeout, [&] { return !m_rosBuffer.empty(); }))
	{
		assert(!m_rosBuffer.empty());

		packet = m_rosBuffer.front();
		m_rosBuffer.pop_front();
	}

	return packet;
}

void MtwCallback::deleteOldestPacket()
{
	XsMutexLocker lock(m_mutex);
	m_packetBuffer.pop_front();
	if (!m_receiveStampBuffer.empty())
		m_receiveStampBuffer.pop_front();
}

int MtwCallback::getMtwIndex() const
{
	return m_mtwIndex;
}

XsDevice const & MtwCallback::device() const
{
	assert(m_device != 0);
	return *m_device;
}

void MtwCallback::onLiveDataAvailable(XsDevice*, const XsDataPacket* packet)
{
	XsMutexLocker lock(m_mutex);
	// NOTE: Processing of packets should not be done in this thread.

	m_packetBuffer.push_back(*packet);
	m_receiveStampBuffer.push_back(m_clock->now());

	if (m_packetBuffer.size() > 300)
	{
		std::cout << std::endl;
		m_packetBuffer.pop_front();
		if (!m_receiveStampBuffer.empty())
			m_receiveStampBuffer.pop_front();
	}
	/*
	assert(packet != 0);

	// Discard oldest packet if buffer full
	if (m_rosBuffer.size() == m_maxBufferSize)
	{
		m_rosBuffer.pop_front();
	}

	// Push new packet
	m_rosBuffer.push_back(RosXsDataPacket(now, *packet));

	// Manual unlocking is done before notifying, to avoid waking up
	// the waiting thread only to block again
	lock.unlock();
	m_condition.notify_one();
	*/
}
