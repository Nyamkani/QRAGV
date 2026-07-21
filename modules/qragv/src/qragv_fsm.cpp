#include <qragv/qragv.hpp>




/* SFM Functions */
void QRAGV::FSMEventCheck()
{
	this->FSMEventRegister();

	for (const auto &eventno: this->sfm_events_) 
	{
		if (eventno != FSMEventNumbering::FSM_EVENT_NONE)
		{
			int eventno_ = eventno;

			this->FSMTransitionState(eventno_);
		}		
	}

	this->sfm_events_.clear();
	
	return;
}

void QRAGV::FSMEventRegister()
{
	/* add events here */
	// this->SFMEventAdd(this->IsIPCInit());
	// this->SFMEventAdd(this->IsETHInit());
	// this->SFMEventAdd(this->IsFatFsInit());
	// this->SFMEventAdd(this->IsAllInit());

	// this->SFMEventAdd(this->IsIPCRun());
	// this->SFMEventAdd(this->IsETHRun());
	// this->SFMEventAdd(this->IsFatFsRun());
	// this->SFMEventAdd(this->IsAllRun());

	// this->SFMEventAdd(this->IsIPCError());
	// this->SFMEventAdd(this->IsETHError());
	// this->SFMEventAdd(this->IsFatFsError());

	// this->SFMEventAdd(this->IsConnected());
	// this->SFMEventAdd(this->IsDisconnected());
    
	/* add events here */
	return;
}

void QRAGV::FSMEventAdd(int event)
{
	const int event_ = event;

	this->sfm_events_.push_back(event_);    
}

/* Specify State Event */
void QRAGV::FSMStateBehavior()
{
	this->state_->StateBehavior(*this);

	return;
}

void QRAGV::FSMTransitionState(int state)
{
	QRAGVState* next_state_ = this->state_->TransitionState(*this, state);

	if (next_state_ == nullptr)
		return;

	this->state_->ExitState(*this);


	delete this->state_;

	this->state_ = next_state_;

	this->state_->EnterState(*this);

	return;
}

void QRAGV::StartInitState()	
{
	this->state_ = new QRAGVInitState();

	this->state_->EnterState(*this);

	return;
}
